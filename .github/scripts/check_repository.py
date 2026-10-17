"""Dependency-free checks for the repository's explicitly selected source and docs."""

import ast
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / ".github/repository-checks.json"


class LocalLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if value and ((tag == "a" and key == "href") or
                          (tag == "img" and key == "src")):
                self.links.append(value)


def selected_files(scopes, suffixes, available):
    result = set()
    for scope in scopes:
        base = ROOT / scope
        if not base.exists():
            raise ValueError(f"Configured path does not exist: {scope}")
        for name in available:
            path = ROOT / name
            if (path == base or base in path.parents) and path.suffix in suffixes:
                result.add(path)
    return sorted(result)


def check_links(path, text):
    # Ignore examples in fenced code; external URLs are intentionally not fetched.
    text = re.sub(r"```.*?```|~~~.*?~~~", "", text, flags=re.DOTALL)
    links = re.findall(r"!?\[[^\]\n]*\]\(([^)\n]+)\)", text)
    html = LocalLinks()
    html.feed(text)
    links.extend(html.links)
    for raw in links:
        url = urlsplit(raw.strip().split()[0].strip("<>"))
        if url.scheme or url.netloc or not url.path:
            continue
        relative = unquote(url.path)
        target = ((ROOT / relative.lstrip("/")) if relative.startswith("/")
                  else path.parent / relative).resolve()
        if not target.is_relative_to(ROOT):
            raise ValueError(f"Local link leaves repository: {raw}")
        if not target.exists():
            raise ValueError(f"Broken local link: {raw}")


def main():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    available = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT,
    ).decode("utf-8").strip("\0").split("\0")
    checks = {
        "python": (set([".py"]), lambda p: ast.parse(p.read_bytes(), filename=str(p))),
        "xml": (set([".xml", ".svg"]), lambda p: ET.parse(p)),
        "markdown": (set([".md"]), lambda p: check_links(p, p.read_text(encoding="utf-8"))),
    }
    errors = []
    for kind, (suffixes, check) in checks.items():
        files = selected_files(config[kind], suffixes, available)
        for path in files:
            try:
                check(path)
            except (SyntaxError, ValueError, OSError, ET.ParseError) as exc:
                errors.append(f"{path.relative_to(ROOT)}: {exc}")
        print(f"{kind}: {len(files)} file(s) checked")
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print("Static checks passed. Runtime, dependencies and hardware are not tested.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
