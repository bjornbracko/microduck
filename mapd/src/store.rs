//! The map on disk.
//!
//! One JSON file, replaced atomically: written beside itself, flushed, renamed over the old one,
//! and the directory flushed, so a power cut leaves either the old map or the new one and never
//! half of either.
//!
//! **A file that cannot be read is set aside, never overwritten.** The first maploc started fresh
//! on any read error and its next autosave replaced the file — a format bump, or a branch build
//! with a different layout, silently destroyed hours of mapping. Here an unreadable map is renamed
//! to `<name>.bad-<seconds>` and said so in the journal, and the new map starts beside it; the old
//! one is still there for whoever wants to know what happened.

use std::io;
use std::path::{Path, PathBuf};

use maploc::mapper::{SESSION_VERSION, Session};

/// What loading found.
#[derive(Debug)]
pub enum Loaded {
    /// A map, to resume.
    Map(Box<Session>),
    /// No file: a robot that has never mapped, or was wiped.
    Nothing,
    /// A file that could not be used, now moved to the path given.
    SetAside { reason: String, moved_to: PathBuf },
}

/// A failed backup is an error: the caller must not start a new map at this path.
pub fn load(path: &Path) -> io::Result<Loaded> {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Loaded::Nothing),
        Err(e) => return set_aside(path, format!("unreadable: {e}")),
    };
    // The version first, on its own: a map from a newer or older format must be named as such,
    // not as whichever field this build happens to find missing.
    #[derive(serde::Deserialize)]
    struct Version {
        version: u32,
    }
    match serde_json::from_slice::<Version>(&bytes) {
        Ok(v) if v.version != SESSION_VERSION => {
            return set_aside(
                path,
                format!(
                    "map format {} (this mapd reads {SESSION_VERSION})",
                    v.version
                ),
            );
        }
        Ok(_) => {}
        Err(e) => return set_aside(path, format!("not a map: {e}")),
    }
    match serde_json::from_slice::<Session>(&bytes) {
        Ok(s) => {
            if s.keyframes.len() != s.graph.nodes.len() || s.island.len() != s.keyframes.len() {
                return set_aside(
                    path,
                    "keyframes, nodes and islands disagree in number".into(),
                );
            }
            if s.graph
                .edges
                .iter()
                .any(|e| e.from >= s.graph.nodes.len() || e.to >= s.graph.nodes.len())
            {
                return set_aside(path, "an edge names a node that does not exist".into());
            }
            Ok(Loaded::Map(Box::new(s)))
        }
        Err(e) => set_aside(path, format!("not a map: {e}")),
    }
}

fn set_aside(path: &Path, reason: String) -> io::Result<Loaded> {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let mut moved_to = path.as_os_str().to_owned();
    moved_to.push(format!(".bad-{secs}"));
    let moved_to = PathBuf::from(moved_to);
    // A new worker saves to the original path. It must not start unless the old
    // file was actually preserved, including when the backup itself is unwritable.
    std::fs::rename(path, &moved_to).map_err(|e| {
        io::Error::new(
            e.kind(),
            format!(
                "{reason}; moving {} aside to {} failed: {e}; refusing to overwrite the saved map",
                path.display(),
                moved_to.display()
            ),
        )
    })?;
    Ok(Loaded::SetAside { reason, moved_to })
}

/// Replace the file with `session`, atomically.
pub fn save(path: &Path, session: &Session) -> std::io::Result<()> {
    let dir = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    {
        // Buffered: serde writes token by token, and unbuffered every one was a syscall — a
        // million of them for a 1.2 MB map, 20 s on a throttled board, with the worker that
        // places stops waiting on it.
        let mut w = std::io::BufWriter::with_capacity(1 << 20, std::fs::File::create(&tmp)?);
        serde_json::to_writer(&mut w, session).map_err(std::io::Error::other)?;
        let f = w.into_inner().map_err(|e| e.into_error())?;
        f.sync_all()?;
    }
    std::fs::rename(&tmp, path)?;
    std::fs::File::open(dir)?.sync_all()
}

/// Forget the map: the file goes, so a restart begins fresh too.
pub fn remove(path: &Path) -> std::io::Result<()> {
    match std::fs::remove_file(path) {
        Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use maploc::mapper::{Mapper, MapperConfig};

    /// How long saving a map of a hundred robot-sized stops takes — a measurement, not a check:
    /// `cargo test --release -p mapd save_speed -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn save_speed() {
        use maploc::keyframe::Keyframe;
        let mut s = Mapper::new(MapperConfig::default()).session();
        for i in 0..100u32 {
            s.keyframes.push(Keyframe {
                id: i,
                odom: maploc::se2::Pose2::IDENTITY,
                t_start_ns: 0,
                t_end_ns: 0,
                res_m: 0.025,
                occ: (0..100).map(|k| [k as i16, -(k as i16)]).collect(),
                free: (0..1500)
                    .map(|k| [(k % 40) as i16, (k / 40) as i16])
                    .collect(),
                n_frames: 50,
                span_rad: 1.0,
            });
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.json");
        let t = std::time::Instant::now();
        save(&path, &s).unwrap();
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let mb = std::fs::metadata(&path).unwrap().len() as f64 / 1e6;
        println!(
            "saved {} keyframes, {mb:.1} MB, in {ms:.0} ms",
            s.keyframes.len()
        );
    }

    #[test]
    fn a_map_survives_the_round_trip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.json");
        let session = Mapper::new(MapperConfig::default()).session();
        save(&path, &session).unwrap();
        match load(&path).unwrap() {
            Loaded::Map(s) => assert_eq!(*s, session),
            other => panic!("{other:?}"),
        }
    }

    /// The first maploc's worst habit: a map it could not read was overwritten by the next
    /// autosave. This one moves it aside and keeps it.
    #[test]
    fn an_unreadable_map_is_kept_aside_not_overwritten() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.json");
        std::fs::write(&path, br#"{"version": 99, "keyframes": []}"#).unwrap();
        let Loaded::SetAside { moved_to, reason } = load(&path).unwrap() else {
            panic!("a future format was loaded");
        };
        assert!(reason.contains("format"), "{reason}");
        assert!(!path.exists());
        assert_eq!(
            std::fs::read(&moved_to).unwrap(),
            br#"{"version": 99, "keyframes": []}"#
        );
        // And a map saved now goes to the original path, beside the kept one.
        save(&path, &Mapper::new(MapperConfig::default()).session()).unwrap();
        assert!(moved_to.exists() && path.exists());
    }

    #[test]
    fn no_file_is_nothing_not_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(matches!(
            load(&dir.path().join("map.json")).unwrap(),
            Loaded::Nothing
        ));
    }
}
