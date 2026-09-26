#!/usr/bin/env python3
"""Download the public CT series used by this tutorial.

Files land under data/chest and data/breathing, next to this tutorial. They are
not committed.
"""

import argparse
from pathlib import Path

# LIDC-IDRI-0001 chest CT, one axial series.
CHEST_SERIES = [
    "1.3.6.1.4.1.14519.5.2.1.6279.6001.179049373636438705059720603192",
]

# 4D-Lung patient 100_HM10395, study S100: ten gated phases, 0% through 90%.
BREATHING_SERIES = [
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.124525254231969293228570633659",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.183508601171753954609852620508",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.187024278831950398228901118188",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.204026634860397031036823480116",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.215193814203822462481389051414",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.303761398024606109161463363548",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.322628904903035357840500590726",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.339352878581931019587697303406",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.742525481756625894234574121639",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.901242277587004706935962916718",
]


def download(dest: Path, series: list[str]) -> None:
    from idc_index import IDCClient

    dest.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {len(series)} series into {dest}")
    IDCClient().download_from_selection(
        str(dest),
        seriesInstanceUID=series,
        dirTemplate="%Modality_%SeriesInstanceUID",
        quiet=False,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset",
        choices=["chest", "breathing", "all"],
        default="all",
        help="Which sample to fetch (default: all)",
    )
    args = parser.parse_args()
    data = Path(__file__).resolve().parents[1] / "data"
    if args.dataset in ("chest", "all"):
        download(data / "chest", CHEST_SERIES)
    if args.dataset in ("breathing", "all"):
        download(data / "breathing", BREATHING_SERIES)


if __name__ == "__main__":
    main()
