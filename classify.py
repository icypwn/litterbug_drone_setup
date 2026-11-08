#!/usr/bin/env python3
"""Ask Gemini what a piece of litter is; its points come from a fixed table per category.

Used by firebase_sync.py on each NEW piece (after landing, before it goes on the map):
    type   one of the consumer app's categories (bottle, can, bag, wrapper, cup, litter)
    points what the app awards for picking it up (POINTS[type]; same table as LitterType.points in the app)

The consumer app uses the same model and categories to check a pickup photo (see VERIFY in the app).

Key: GEMINI_API_KEY in the environment, or secrets/gemini_key.txt next to this file.

  python classify.py photo.jpg [more.jpg ...]     # try it on some crops
"""
import base64, json, logging, os, sys, time, urllib.error, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.environ.get("LITTERBUG_GEMINI_MODEL", "gemini-3.7-flash")
TYPES = ["bottle", "can", "bag", "wrapper", "cup", "litter"]   # same list as LitterType.all in the app
# Fixed points per category, by how much good removing it does (keep in sync with LitterType.points in the app).
POINTS = {"bag": 80, "bottle": 80, "can": 60, "cup": 60, "wrapper": 50, "litter": 30}
FALLBACK = {"type": "litter", "points": POINTS["litter"], "reason": "not classified"}
log = logging.getLogger("classify")

PROMPT = """This is a photo of one piece of litter, taken from a drone about 15 feet up (it may be small or blurry).

1. type: which ONE category it is:
   bottle  - plastic or glass drink bottle
   can     - metal drink or food can
   bag     - plastic or paper bag
   wrapper - snack, candy or food wrapper, chip packet
   cup     - disposable cup, coffee cup, cup lid
   litter  - anything else, or you can't tell
2. reason: at most 8 words on what you see.
"""

SCHEMA = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "enum": TYPES},
        "reason": {"type": "string"},
    },
    "required": ["type", "reason"],
}


def api_key():
    key = os.environ.get("GEMINI_API_KEY")
    if not key:
        try:
            key = open(os.path.join(HERE, "secrets", "gemini_key.txt")).read().strip()
        except OSError:
            key = None
    return key


def classify_jpeg(jpeg_bytes, key=None, retries=3):
    """-> {"type", "points", "reason"}. Raises if Gemini can't be reached (caller retries later)."""
    key = key or api_key()
    if not key:
        raise RuntimeError("no Gemini key (set GEMINI_API_KEY or secrets/gemini_key.txt)")
    body = json.dumps({
        "contents": [{"role": "user", "parts": [
            {"inline_data": {"mime_type": "image/jpeg", "data": base64.b64encode(jpeg_bytes).decode()}},
            {"text": PROMPT},
        ]}],
        "generationConfig": {"temperature": 0, "responseMimeType": "application/json", "responseSchema": SCHEMA},
    }).encode()
    req = urllib.request.Request(
        f"https://generativelanguage.googleapis.com/v1beta/models/{MODEL}:generateContent",
        data=body, headers={"Content-Type": "application/json", "x-goog-api-key": key})
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                reply = json.load(r)
            parts = ((reply.get("candidates") or [{}])[0].get("content") or {}).get("parts") or []
            out = json.loads("".join(p.get("text", "") for p in parts))
            t = out.get("type") if out.get("type") in TYPES else "litter"
            return {"type": t, "points": POINTS[t], "reason": str(out.get("reason", ""))[:80]}
        except urllib.error.HTTPError as e:
            if e.code in (429, 500, 502, 503) and attempt < retries - 1:
                time.sleep(2 ** attempt + 0.5)
                continue
            raise RuntimeError(f"Gemini HTTP {e.code}: {e.read()[:200]!r}")
        except (ValueError, KeyError) as e:   # unusable reply; try again
            if attempt == retries - 1:
                raise RuntimeError(f"Gemini reply unusable: {e}")


def classify_file(path, key=None):
    with open(path, "rb") as f:
        return classify_jpeg(f.read(), key)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    for p in sys.argv[1:]:
        print(p, classify_file(p))
