#!/usr/bin/env python3
"""Render the manual (cover + body) to one PDF with playwright/chromium.
Usage: python3 build_manual.py [output.pdf]
Screenshots are expected in docs/manual/shots (see e2e/doc/doc_screenshots.py)."""
import pathlib, sys
from playwright.sync_api import sync_playwright
from pypdf import PdfReader, PdfWriter

here = pathlib.Path(__file__).resolve().parent
out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else here / "SDRPP-TETRA-SCHWARTZ_Bedienungsanleitung.pdf")
tmp = here / "_build"
tmp.mkdir(exist_ok=True)

footer = ('<div style="width:100%;font-family:Inter,sans-serif;font-size:7.5pt;color:#5a6774;'
          'padding:0 17mm;display:flex;justify-content:space-between">'
          '<span>SDR++ TETRA-SCHWARTZ · Bedienungsanleitung</span>'
          '<span><span class="pageNumber"></span></span></div>')

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page()
    pg.goto((here / "manual_cover.html").as_uri()); pg.wait_for_load_state("networkidle")
    pg.pdf(path=str(tmp / "cover.pdf"), format="A4", print_background=True, prefer_css_page_size=True)
    pg.goto((here / "manual_body.html").as_uri()); pg.wait_for_load_state("networkidle")
    pg.pdf(path=str(tmp / "body.pdf"), format="A4", print_background=True, prefer_css_page_size=True,
           display_header_footer=True, header_template="<div></div>", footer_template=footer)
    b.close()

w = PdfWriter()
for f in ("cover.pdf", "body.pdf"):
    for page in PdfReader(str(tmp / f)).pages:
        w.add_page(page)
w.add_metadata({"/Title": "SDR++ TETRA-SCHWARTZ – Bedienungsanleitung", "/Author": "UnImAtrIXzEr0",
                "/Subject": "Bedienungsanleitung v0.19.0-beta", "/Creator": "UnImAtrIXzEr0"})
w.page_mode = "/UseOutlines"
with open(out, "wb") as fh:
    w.write(fh)
print(out, len(w.pages), "pages")
