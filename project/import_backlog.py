#!/usr/bin/env python3
"""
Imports backlog.csv as GitHub issues and adds them to a Project.
  Title,Body,Labels,Assignees,Status,Epic,Priority,Sprint

Usage:
    python import_backlog.py backlog.csv

Rules:
  • If an issue with the same title already exists (case-insensitive), reuse it.
  • Otherwise, create the issue, create any missing labels, apply labels/assignees.
  • In both cases, add the issue to the Project (ignore if already present).

Requirements:
  • GitHub CLI ≥ 2.74 already authenticated (`gh auth status`)
  • Python ≥ 3.8

Example:
python import_backlog.py backlog.csv

"""

import csv, json, subprocess, sys, textwrap

# --------- CONFIGURE HERE ------------------------------------
OWNER   = "thiagolages"
REPO    = "robotic-manipulators"
PROJECT = 2                       # Project number
CSVPATH = sys.argv[1]             # backlog.csv
# -------------------------------------------------------------


def run(cmd):
    """Run shell command and return stdout; abort on error."""
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr, end="")
        sys.exit(proc.returncode)
    return proc.stdout.strip()


def ensure_label(label: str):
    """Create label if it doesn't exist."""
    existing = run(
        ["gh", "label", "list", "--repo", f"{OWNER}/{REPO}", "--limit", "500", "--json", "name"]
    )
    if not any(d["name"].lower() == label.lower() for d in json.loads(existing)):
        run(
            [
                "gh",
                "label",
                "create",
                label,
                "--repo",
                f"{OWNER}/{REPO}",
                "--color",
                "ededed",
            ]
        )


def find_existing_issue(title: str) -> str | None:
    """Return URL of existing issue with exact same title (case-insensitive), else None."""
    data = run(
        [
            "gh",
            "issue",
            "list",
            "--repo",
            f"{OWNER}/{REPO}",
            "--search",
            f'"{title}" in:title',
            "--state",
            "all",
            "--json",
            "title,url",
            "--limit",
            "100",
        ]
    )
    for item in json.loads(data):
        if item["title"].lower() == title.lower():
            return item["url"]
    return None


def add_to_project(issue_url: str):
    """Add issue URL to project; ignore if already present."""
    try:
        run(
            [
                "gh",
                "project",
                "item-add",
                str(PROJECT),
                "--owner",
                OWNER,
                "--url",
                issue_url,
            ]
        )
    except SystemExit:
        # likely already exists in project; ignore
        pass


with open(CSVPATH, newline="", encoding="utf-8") as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        title        = row["Title"].strip()
        body         = row["Body"].strip()
        labels_raw   = row["Labels"].strip()
        assignees_raw = row["Assignees"].strip()

        # 1) Does it exist?
        issue_url = find_existing_issue(title)
        if issue_url:
            add_to_project(issue_url)
            print(f"↺ {title}")
            continue

        # 2) Prepare labels/assignees
        label_flags = []
        if labels_raw:
            for lbl in [l.strip() for l in labels_raw.split(",") if l.strip()]:
                ensure_label(lbl)
                label_flags += ["--label", lbl]

        assignee_flags = []
        if assignees_raw:
            for user in [u.strip() for u in assignees_raw.split(",") if u.strip()]:
                assignee_flags += ["--assignee", user]

        # 3) Create issue
        issue_url = run(
            [
                "gh",
                "issue",
                "create",
                "--repo",
                f"{OWNER}/{REPO}",
                "--title",
                title,
                "--body",
                body or " ",
                *label_flags,
                *assignee_flags,
            ]
        ).split()[-1]  # URL is the last word

        # 4) Add to Project
        add_to_project(issue_url)
        print(f"✔︎ {title}")

print("\nDone.")
