#!/usr/bin/env python3
"""Ask Gemini what a piece of litter is and how much picking it up helps the environment.

Used by firebase_sync.py on each NEW piece (after landing, before it goes on the map):
    type   one of the consumer app's categories (bottle, can, bag, wrapper, cup, litter)
    score  1-10, how much good removing it does (harm to wildlife and water, how long it lasts, hazard)
    points what the app awards for picking it up (score x POINTS_PER_SCORE)

The consumer app uses the same model and categories to check a pickup photo (see VERIFY in the app).

Key: GEMINI_API_KEY in the environment, or secrets/gemini_key.txt next to this file.

  python classify.py photo.jpg [more.jpg ...]     # try it on some crops
"""
import base64, json, logging, os, sys, time, urllib.error, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.environ.get("LITTERBUG_GEMINI_MODEL", "gemini-3.7-flash")
TYPES = ["bottle", "can", "bag", "wrapper", "cup", "litter"]   # same list as LitterType.all in the app
POINTS_PER_SCORE = 10
FALLBACK = {"type": "litter", "score": 3, "points": 3 * POINTS_PER_SCORE, "reason": "not classified"}
log = logging.getLogger("classify")

PROMPT = """This is a photo of one piece of litter, taken from a drone about 15 feet up (it may be small or blurry).

1. type: which ONE category it is:
   bottle  - plastic or glass drink bottle
   can     - metal drink or food can
   bag     - plastic or paper bag
   wrapper - snack, candy or food wrapper, chip packet
   cup     - disposable cup, coffee cup, cup lid
   litter  - anything else, or you can't tell
2. score: 1-10, how much good it does the environment to pick this up. Weigh: harm to wildlife (entanglement,
   being eaten), whether it breaks into microplastics or leaches, how long it would last outdoors, and hazards
   (broken glass, sharp metal). E.g. a plastic bag or bottle scores high; a paper napkin scores low.
3. reason: at most 8 words explaining the score.
"""

SCHEMA = {
    "type": "object",
    "properties": {
        "type": {"type": "string", "enum": TYPES},
        "score": {"type": "integer", "minimum": 1, "maximum": 10},
        "reason": {"type": "string"},
    },
    "required": ["type", "score", "reason"],
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
    """-> {"type", "score", "points", "reason"}. Raises if Gemini can't be reached (caller retries later)."""
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
            score = min(10, max(1, int(out.get("score", FALLBACK["score"]))))
            return {"type": t, "score": score, "points": score * POINTS_PER_SCORE, "reason": str(out.get("reason", ""))[:80]}
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
