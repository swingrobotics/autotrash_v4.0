"""PC-side compute worker for SWING_CAR.

Keep package import lightweight so the Windows Manager can start without
loading the training stack. Public worker symbols remain available lazily for
backward compatibility.
"""

__all__ = ["ComputeWorker", "PipelineComputeWorker", "WorkerConfig", "main"]
__version__ = "0.3.0"


def __getattr__(name):
    if name in {"PipelineComputeWorker", "main"}:
        from .pipeline_worker import PipelineComputeWorker, main

        return {"PipelineComputeWorker": PipelineComputeWorker, "main": main}[name]
    if name in {"ComputeWorker", "WorkerConfig"}:
        from .worker import ComputeWorker, WorkerConfig

        return {"ComputeWorker": ComputeWorker, "WorkerConfig": WorkerConfig}[name]
    raise AttributeError(name)
