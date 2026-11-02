"""Build vslam_examples/STUDENT_GUIDE.html from STUDENT_GUIDE.md.

The HTML file is self-contained: the figures are embedded, so students can
open it in any web browser (Windows or Mac) without other files.

Usage (needs the Python package "markdown": pip install markdown):
    python tools/build_guide_html.py
"""
import base64
import pathlib
import re

import markdown

ROOT = pathlib.Path(__file__).resolve().parent.parent
GUIDE_DIR = ROOT / "vslam_examples"
SOURCE = GUIDE_DIR / "STUDENT_GUIDE.md"
TARGET = GUIDE_DIR / "STUDENT_GUIDE.html"

CSS = """
body { font-family: -apple-system, "Segoe UI", Helvetica, Arial, sans-serif;
       max-width: 980px; margin: 2em auto; padding: 0 1em; line-height: 1.5; color: #1f2328; }
h1, h2 { border-bottom: 1px solid #d0d7de; padding-bottom: .3em; }
h3 { margin-top: 1.6em; }
table { border-collapse: collapse; margin: 1em 0; }
th, td { border: 1px solid #d0d7de; padding: 6px 10px; vertical-align: top; }
th { background: #f6f8fa; }
code { background: #f6f8fa; padding: .1em .3em; border-radius: 4px; }
pre { background: #f6f8fa; padding: 1em; border-radius: 6px; overflow-x: auto; }
pre code { padding: 0; }
blockquote { border-left: 4px solid #d29922; background: #fff8c5; margin: 1em 0; padding: .5em 1em; }
img { max-width: 100%; border: 1px solid #d0d7de; }
"""


def embed_images(html: str) -> str:
    """Replace each local image path with an embedded data URI."""
    def repl(match):
        path = GUIDE_DIR / match.group(1)
        data = base64.b64encode(path.read_bytes()).decode("ascii")
        return f'src="data:image/png;base64,{data}"'
    return re.sub(r'src="(docs/images/[^"]+\.png)"', repl, html)


def main():
    text = SOURCE.read_text(encoding="utf-8")
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "toc", "sane_lists"])
    body = embed_images(body)
    title = text.splitlines()[0].lstrip("# ").strip()
    page = (f"<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
            f"<title>{title}</title>\n<style>{CSS}</style>\n</head>\n<body>\n"
            f"<!-- Generated from STUDENT_GUIDE.md by tools/build_guide_html.py. Do not edit. -->\n"
            f"{body}\n</body>\n</html>\n")
    TARGET.write_text(page, encoding="utf-8")
    print(f"Wrote {TARGET} ({len(page) // 1024} KB)")


if __name__ == "__main__":
    main()
