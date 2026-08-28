import argparse
import json

from .log_replay import LogReplay


def main():
    parser = argparse.ArgumentParser(description="Inspect an autonomous driving recording")
    parser.add_argument("session_path")
    parser.add_argument("--at", type=float, help="Show the latest stream values at a monotonic timestamp")
    arguments = parser.parse_args()
    replay = LogReplay(arguments.session_path)
    document = replay.summary()
    if arguments.at is not None:
        document["state_at"] = replay.state_at(arguments.at)
    print(json.dumps(document, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
