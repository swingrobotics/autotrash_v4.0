"""AUTO_AI training dataset split policy.

The generic dataset builder uses 75/12.5/12.5 whole-session splits. With the
BASE training minimum of exactly three RECORD sessions, that rounds to one
train, one validation and one test session. That technically preserves all
three buckets but leaves the model learning from only one driving session.

For the explicit three-session BASE minimum, prefer two train sessions and one
validation session. The training pipeline already falls back to validation as
the held-out evaluation split when no test session exists. Four or more
sessions retain the generic whole-session split policy unchanged.
"""

from __future__ import annotations

from .frame_dataset_builder import DatasetBuilder as _FrameDatasetBuilder


class DatasetBuilder(_FrameDatasetBuilder):
    """Frame-aware builder with a sane split for the 3-session BASE minimum."""

    def _assign_session_splits(self, sessions):
        result = super()._assign_session_splits(sessions)
        if len(result) == 3:
            test_sessions = [
                session for session, split in result.items() if split == "test"
            ]
            # The generic splitter is deterministic and produces exactly one
            # test session for count=3. Convert only that held-out bucket to
            # training, preserving the existing validation session.
            if len(test_sessions) == 1:
                result[test_sessions[0]] = "train"
        return result


__all__ = ["DatasetBuilder"]
