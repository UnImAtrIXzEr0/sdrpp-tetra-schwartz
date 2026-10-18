#!/usr/bin/env python3
"""Render the manual (cover + body) to PDF with playwright/chromium, German and English.
Usage: python3 build_manual.py [output folder]
Screenshots are expected in docs/manual/shots (branch doc-shots, see e2e/doc/doc_screenshots.py)."""
import pathlib, sys
from playwright.sync_api import sync_playwright
from pypdf import PdfReader, PdfWriter

here = pathlib.Path(__file__).resolve().parent
outdir = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else here)
tmp = here / "_build"
tmp.mkdir(exist_ok=True)
VERSION = "v0.20.2-beta"

LANGS = {
    "de": dict(cover="manual_cover.html", body="manual_body.html",
               out=f"SDRPP-TETRA-SCHWARTZ_Bedienungsanleitung_{VERSION}.pdf",
               title="SDR++ TETRA-SCHWARTZ – Bedienungsanleitung", subject=f"Bedienungsanleitung {VERSION}",
               foot="SDR++ TETRA-SCHWARTZ · Bedienungsanleitung · nur für experimentelle Amateurfunkzwecke"),
    "en": dict(cover="manual_cover_en.html", body="manual_body_en.html",
               out=f"SDRPP-TETRA-SCHWARTZ_User_Manual_{VERSION}.pdf",
               title="SDR++ TETRA-SCHWARTZ – User manual", subject=f"User manual {VERSION}",
               foot="SDR++ TETRA-SCHWARTZ · User manual · for experimental amateur radio purposes only"),
}

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page()
    for lang, c in LANGS.items():
        footer = ('<div style="width:100%;font-family:Inter,sans-serif;font-size:7.5pt;color:#5a6774;'
                  'padding:0 17mm;display:flex;justify-content:space-between">'
                  f'<span>{c["foot"]}</span><span><span class="pageNumber"></span></span></div>')
        pg.goto((here / c["cover"]).as_uri()); pg.wait_for_load_state("networkidle")
        pg.pdf(path=str(tmp / f"cover_{lang}.pdf"), format="A4", print_background=True, prefer_css_page_size=True)
        pg.goto((here / c["body"]).as_uri()); pg.wait_for_load_state("networkidle")
        pg.pdf(path=str(tmp / f"body_{lang}.pdf"), format="A4", print_background=True, prefer_css_page_size=True,
               display_header_footer=True, header_template="<div></div>", footer_template=footer)
        w = PdfWriter()
        for f in (f"cover_{lang}.pdf", f"body_{lang}.pdf"):
            for page in PdfReader(str(tmp / f)).pages:
                w.add_page(page)
        w.add_metadata({"/Title": c["title"], "/Author": "UnImAtrIXzEr0", "/Subject": c["subject"], "/Creator": "UnImAtrIXzEr0"})
        out = outdir / c["out"]
        with open(out, "wb") as fh:
            w.write(fh)
        print(out, len(w.pages), "pages")
    b.close()
