import json
import os


ALLOWED_SPLITS = {"train", "validation", "test"}


def validate_dataset_split_integrity(dataset_path, document=None):
    """Reject session/frame leakage across train/validation/test.

    Dataset builders already assign complete RECORD sessions to one split. This
    second validation lives at the training/runtime boundary so manually edited
    or externally generated manifests cannot silently place adjacent frames from
    the same drive in both training and held-out evaluation.
    """
    dataset_path = os.path.abspath(dataset_path)
    if document is None:
        with open(os.path.join(dataset_path, "dataset.json"), "r", encoding="utf-8") as file:
            document = json.load(file)
    manifest_path = os.path.join(
        dataset_path,
        document.get("sample_manifest", "samples.jsonl"),
    )

    session_splits = {}
    source_splits = {}
    split_counts = {name: 0 for name in ALLOWED_SPLITS}
    total = 0
    with open(manifest_path, "r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            sample = json.loads(line)
            total += 1
            split = str(sample.get("split") or "").strip().lower()
            if split not in ALLOWED_SPLITS:
                raise ValueError(
                    f"Invalid dataset split at {os.path.basename(manifest_path)}:{line_number}: {split!r}"
                )
            split_counts[split] += 1

            session = str(sample.get("session") or "").strip()
            if session:
                previous = session_splits.setdefault(session, split)
                if previous != split:
                    raise ValueError(
                        "Dataset leakage detected: session "
                        f"{session!r} appears in both {previous!r} and {split!r}"
                    )

            camera = sample.get("camera") or {}
            sequence = camera.get("source_sequence")
            if sequence is None:
                sequence = camera.get("video_frame_index")
            timestamp = sample.get("timestamp_monotonic")
            if session and sequence is not None:
                source_key = (session, "sequence", str(sequence))
            elif session and timestamp is not None:
                source_key = (session, "timestamp", str(timestamp))
            else:
                source_key = None
            if source_key is not None:
                previous = source_splits.setdefault(source_key, split)
                if previous != split:
                    raise ValueError(
                        "Dataset leakage detected: the same camera source appears "
                        f"in both {previous!r} and {split!r}: {source_key}"
                    )

    return {
        "samples": total,
        "sessions": len(session_splits),
        "split_counts": split_counts,
        "session_isolation_verified": True,
    }


__all__ = ["ALLOWED_SPLITS", "validate_dataset_split_integrity"]
