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

# 4D-Lung patient 100_HM10395, study S300: ten gated phases, 0% through 90%.
# About 75 MB per phase, ~750 MB total.
BREATHING_SERIES = [
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.660205801121995704108596829193",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.124579141450014659493072582400",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.238102828057750190064634581534",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.104367227242206784402589566113",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.346792788302694778312512645229",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.235823857759482707932854857212",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.211370457211109403388277199194",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.309178825287344133434085485451",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.103521604337852189272990015277",
    "1.3.6.1.4.1.14519.5.2.1.6834.5010.334245065657098504984831070956",
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
