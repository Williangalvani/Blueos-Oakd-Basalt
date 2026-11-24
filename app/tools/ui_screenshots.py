#!/usr/bin/env python3
"""
Screenshots of the web UI with headless Chrome over the DevTools protocol (needs websocket-client), against
a local server, normally tools/ui_demo_server.py:

    python app/tools/ui_demo_server.py --port 8099 &
    python app/tools/ui_demo_server.py --port 8098 --armed &
    python app/tools/ui_screenshots.py --url http://127.0.0.1:8099/ --out ~/oakd-stage1/ui-impl
    python app/tools/ui_screenshots.py --url http://127.0.0.1:8098/ --out ~/oakd-stage1/ui-impl --scenario armed

main: the full page with live views refreshed and a depth probe (main.png), the parameter write confirm
dialog (confirm-dialog.png), its result after confirming against the fake autopilot (write-result.png), a
settings validation error (settings-error.png) and the page after Stop (vio-stopped.png).
armed: the confirm dialog and the server's refusal while the fake autopilot is armed (armed-refusal.png).
live: against a mock server (main.py --mock), the Live camera panel streaming with the tracked features
drawn on it (live-camera.png) and the Cockpit widget (widget-camera.png).
link: against a mock server, the status panel with the Camera link box and its per-stream table while the
live stream runs (link-panel.png), and the widget's status line with the link (widget-link.png).
"""

import argparse
import base64
import json
import shutil
import subprocess
import tempfile
import time
import urllib.parse
import urllib.request
from pathlib import Path

import websocket

CHROME = "/opt/google/chrome/chrome"
WIDTH = 1400


class Browser:
    def __init__(self, chrome=CHROME, port=9333):
        self.profile = tempfile.mkdtemp(prefix="oakd-ui-chrome-")
        self.process = subprocess.Popen(
            [chrome, "--headless=new", "--no-sandbox", "--disable-gpu", "--hide-scrollbars",
             f"--remote-debugging-port={port}", f"--user-data-dir={self.profile}", f"--window-size={WIDTH},1000",
             "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.time() + 15
        while True:
            try:
                targets = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list", timeout=1).read())
                page = next(t for t in targets if t["type"] == "page")
                break
            except Exception:
                if time.time() > deadline:
                    raise
                time.sleep(0.2)
        self.ws = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=30, suppress_origin=True)
        self.next_id = 0

    def send(self, method, **params):
        self.next_id += 1
        self.ws.send(json.dumps({"id": self.next_id, "method": method, "params": params}))
        while True:
            reply = json.loads(self.ws.recv())
            if reply.get("id") == self.next_id:
                if "error" in reply:
                    raise RuntimeError(f"{method}: {reply['error']}")
                return reply.get("result", {})

    def js(self, expression):
        result = self.send("Runtime.evaluate", expression=expression, awaitPromise=True, returnByValue=True)
        if "exceptionDetails" in result:
            raise RuntimeError(f"{expression}: {result['exceptionDetails']}")
        return result.get("result", {}).get("value")

    def wait_for(self, expression, timeout=10.0):
        deadline = time.time() + timeout
        while not self.js(expression):
            if time.time() > deadline:
                raise TimeoutError(expression)
            time.sleep(0.2)

    def viewport(self, height):
        self.send("Emulation.setDeviceMetricsOverride", width=WIDTH, height=int(height), deviceScaleFactor=1,
                  mobile=False)

    def screenshot(self, path, full=False):
        if full:
            self.viewport(self.js("document.documentElement.scrollHeight"))
            time.sleep(1.0)
        data = self.send("Page.captureScreenshot", format="png", captureBeyondViewport=full)["data"]
        Path(path).write_bytes(base64.b64decode(data))
        print(f"wrote {path}")
        if full:
            self.viewport(1000)

    def close(self):
        try:
            self.ws.close()
        finally:
            self.process.terminate()
            self.process.wait(10)
            shutil.rmtree(self.profile, ignore_errors=True)


def open_page(browser, url):
    browser.viewport(1000)
    browser.send("Page.enable")
    browser.send("Page.navigate", url=url)
    browser.wait_for("document.readyState === 'complete'")
    browser.wait_for("document.querySelectorAll('#ap-table tbody tr[data-name]').length > 0", timeout=20)
    browser.wait_for("document.querySelector('#plot-vxvy path') !== null", timeout=20)


def scenario_main(browser, url, out):
    open_page(browser, url)
    browser.js("document.getElementById('probe-x').value = 330; document.getElementById('probe-y').value = 230;"
               "document.getElementById('view-crosshair').checked = true;"
               "document.getElementById('view-auto').checked = true; refreshViews(true)")
    browser.wait_for("document.querySelector('#view-right img').getAttribute('src') !== null", timeout=15)
    browser.js("depthProbe()")
    browser.wait_for("document.getElementById('probe-summary').textContent.startsWith('→')", timeout=20)
    time.sleep(25.0)  # the DVL overlay starts when the page first asks for history
    browser.screenshot(out / "main.png", full=True)

    browser.js("document.getElementById('view-auto').checked = false; window.scrollTo(0, 0); openWriteDialog()")
    time.sleep(0.5)
    browser.screenshot(out / "confirm-dialog.png")
    browser.js("confirmWrite()")
    browser.wait_for("document.getElementById('ap-dialog-result').textContent.length > 0", timeout=20)
    time.sleep(0.3)
    browser.screenshot(out / "write-result.png")
    browser.js("closeWriteDialog()")

    browser.js("const f = document.getElementById('config-form'); f.elements.fps.value = '0';"
               "f.elements.vehicle_ip.value = 'bad host'; f.requestSubmit()")
    time.sleep(0.5)
    browser.js("document.getElementById('panel-settings').scrollIntoView()")
    time.sleep(0.3)
    browser.screenshot(out / "settings-error.png")
    browser.send("Page.reload")
    browser.wait_for("document.readyState === 'complete'")

    browser.js("stopVIO()")
    time.sleep(4.0)
    browser.js("window.scrollTo(0, 0)")
    browser.screenshot(out / "vio-stopped.png")
    browser.js("startVIO()")


def scenario_armed(browser, url, out):
    open_page(browser, url)
    browser.js("window.scrollTo(0, 0); openWriteDialog()")
    time.sleep(0.5)
    browser.js("confirmWrite()")
    browser.wait_for("document.getElementById('ap-dialog-result').textContent.length > 0", timeout=20)
    time.sleep(0.3)
    browser.screenshot(out / "armed-refusal.png")


def scenario_live(browser, url, out):
    browser.viewport(1000)
    browser.send("Page.enable")
    browser.send("Page.navigate", url=url)
    browser.wait_for("document.readyState === 'complete'")
    browser.js("document.getElementById('depth-stream-on').checked = true; toggleDepthStream();"
               "document.getElementById('panel-depth-stream').scrollIntoView()")
    browser.wait_for("document.getElementById('depth-stream-img').naturalWidth > 1", timeout=20)
    browser.wait_for("depthStream.features && depthStream.features.points.length > 0", timeout=20)
    time.sleep(2.0)
    browser.js("document.getElementById('panel-depth-stream').scrollIntoView()")
    browser.screenshot(out / "live-camera.png")
    browser.js("document.getElementById('depth-stream-on').checked = false; toggleDepthStream()")
    browser.send("Emulation.setDeviceMetricsOverride", width=800, height=520, deviceScaleFactor=1, mobile=False)
    browser.send("Page.navigate", url=urllib.parse.urljoin(url, "widget/depth"))
    browser.wait_for("document.readyState === 'complete'")
    browser.wait_for("w.frame !== null && w.features !== null", timeout=20)
    time.sleep(2.0)
    browser.screenshot(out / "widget-camera.png")


def scenario_link(browser, url, out):
    browser.viewport(1000)
    browser.send("Page.enable")
    browser.send("Page.navigate", url=url)
    browser.wait_for("document.readyState === 'complete'")
    browser.js("document.getElementById('depth-stream-on').checked = true; toggleDepthStream()")
    browser.wait_for("[...document.querySelectorAll('#link-streams-body td:first-child')]"
                     ".some(td => td.textContent.startsWith('depth_stream'))", timeout=30)
    time.sleep(1.5)
    box = browser.js("JSON.stringify(document.getElementById('panel-status').getBoundingClientRect())")
    box = json.loads(box)
    browser.viewport(max(1000, box["y"] + box["height"] + 20))
    time.sleep(0.5)
    data = browser.send("Page.captureScreenshot", format="png", clip={
        "x": box["x"], "y": box["y"], "width": box["width"], "height": box["height"], "scale": 1})["data"]
    (out / "link-panel.png").write_bytes(base64.b64decode(data))
    print(f"wrote {out / 'link-panel.png'}")
    browser.js("document.getElementById('depth-stream-on').checked = false; toggleDepthStream()")
    browser.send("Emulation.setDeviceMetricsOverride", width=1000, height=520, deviceScaleFactor=1, mobile=False)
    browser.send("Page.navigate", url=urllib.parse.urljoin(url, "widget/depth"))
    browser.wait_for("document.readyState === 'complete'")
    browser.wait_for("document.getElementById('status-line').textContent.includes('link ')", timeout=20)
    time.sleep(1.0)
    browser.screenshot(out / "widget-link.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="http://127.0.0.1:8099/")
    parser.add_argument("--out", type=Path, default=Path.home() / "oakd-stage1/ui-impl")
    parser.add_argument("--scenario", choices=("main", "armed", "live", "link"), default="main")
    parser.add_argument("--chrome", default=CHROME)
    parser.add_argument("--debug-port", type=int, default=9333)
    args = parser.parse_args()
    host = urllib.parse.urlparse(args.url).hostname
    if host not in ("127.0.0.1", "localhost"):
        raise SystemExit("screenshots are only taken of a local (demo) server")
    args.out = args.out.expanduser()
    args.out.mkdir(parents=True, exist_ok=True)
    browser = Browser(args.chrome, args.debug_port)
    try:
        scenarios = {"main": scenario_main, "armed": scenario_armed, "live": scenario_live, "link": scenario_link}
        scenarios[args.scenario](browser, args.url, args.out)
    finally:
        browser.close()


if __name__ == "__main__":
    main()
