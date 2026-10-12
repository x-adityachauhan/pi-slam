"""Create CPU-only run settings without modifying camera intrinsics or GPU files."""
import argparse
from pathlib import Path
import re


def configure(text, profile):
    parameters = {"baseline": (1600, 20, 7), "candidate": (2000, 12, 5)}
    count, initial, minimum = parameters[profile]
    for key, value in {"ORBextractor.nFeatures": count, "ORBextractor.iniThFAST": initial,
                       "ORBextractor.minThFAST": minimum}.items():
        text, n = re.subn(r"(?m)^"+re.escape(key)+r":.*$", f"{key}: {value}", text)
        if n != 1:
            raise ValueError(f"Missing or repeated setting {key}")
    text = re.sub(r"(?m)^CPU.VisualRecoverySeconds:.*\n?", "", text)
    return text.rstrip() + "\n\nCPU.VisualRecoverySeconds: 15.0\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("--profile", choices=["baseline", "candidate"], default="baseline")
    args = parser.parse_args()
    args.target.write_text(configure(args.source.read_text(), args.profile))
