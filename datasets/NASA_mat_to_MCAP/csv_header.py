"""
    This script is used to add the header to each CSV file
"""

import os

WORKDIR = os.path.dirname(os.path.abspath(__file__))
CSV_ROOT = os.path.join(WORKDIR, "csv")
CSV_HEADERS_DIR = os.path.join(WORKDIR, "csv_headers")


def generate_csv_headers() -> None:

    if not os.path.exists(CSV_HEADERS_DIR):
        os.mkdir(CSV_HEADERS_DIR)

    files = os.listdir(CSV_ROOT)
    for file in files:
        header = [file.replace(".csv", "")+'\n']
        print(header)
        with open(os.path.join(CSV_ROOT, file), encoding="utf-8") as f:
            lines = f.readlines()

        header.extend(lines)

        with open(os.path.join(CSV_HEADERS_DIR, file), "w", encoding="utf-8") as f:
            f.writelines(header)
