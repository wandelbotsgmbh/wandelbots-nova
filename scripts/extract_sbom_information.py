"""
Usage: uvx cyclonedx-py environment --of JSON | uv run python ./scripts/extract_sbom_information.py
"""

import csv
import json
import sys


def extract_components(data):
    rows = []

    for comp in data.get("components", []):
        name = comp.get("name")
        version = comp.get("version")
        purl = comp.get("purl")

        license_id = None
        licenses = comp.get("licenses", [])

        if licenses:
            lic_entry = licenses[0]

            if "license" in lic_entry:
                license_obj = lic_entry["license"]
                license_id = license_obj.get("id") or license_obj.get("name")
            else:
                license_id = lic_entry.get("id") or lic_entry.get("expression")

        rows.append([name, version, license_id, purl])

    return rows


if __name__ == "__main__":
    # read JSON from stdin (pipe)
    data = json.load(sys.stdin)

    rows = extract_components(data)

    writer = csv.writer(sys.stdout)

    writer.writerow(["name", "version", "license", "purl"])
    writer.writerows(rows)
