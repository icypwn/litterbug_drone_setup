# Field test checklist

Everything below is changed on the operator page: **Settings** tab (or open `http://<pi>:8080/#settings`).
Settings save by themselves and apply to the next camera test or mission.

## Before every test
1. **Settings → This test → Test name / notes**: say what you're testing, e.g. `T3 conf 0.25, grass field`.
   It's saved with the run and shown under Flights.
2. **Upload to Firebase: off** for all tests. Those runs are never published, even if you switch it back on.
3. Put out litter you can count: write down how many pieces and roughly where.
   Include **a pair 30–50 cm apart** and **one item in the overlap between two passes**.
4. After landing: **Flights → the run**. Check the count against yours, then check the crops and the replay.

## Tests, in order
| # | What | Setting(s) to change | Look for |
|---|---|---|---|
| 1 | Camera direction | none; camera test, walk forward holding the drone | Ground moves **down** the picture. If not, change *Top of the image points to the drone's*. **Do this first: wrong = tracking breaks.** |
| 2 | Sharp pictures | *Fast shutter* on/off, *Lock focus* = your height | Crops sharp while moving? Too dark with fast shutter → try 2000 µs. |
| 3 | Baseline flight | defaults | Pieces found vs your count. Save as preset "Baseline". |
| 4 | Find more | *Confidence threshold* 0.25 | More real pieces? How many false ones? |
| 5 | Fewer false alarms | *Frames before a piece goes on the map* 4–5 | False ones gone, real ones kept? |
| 6 | Double boxes | *Drop double boxes* on (0.85) | Fewer duplicate pins on bottles and caps? Close pair still 2? |
| 7 | Height / speed | Map tab: height 12 / 15 / 20 ft, speed 1.5 / 2 / 3 m/s | Count and crops at each. Set *Lock focus* to the new height. |

## Tuning tracking without flying again
**Flights → a run → Re-run tracking with other settings**: change *match distance*, *forget after*,
*same piece on another pass*, etc., then press **Re-run**. It shows the new count and blue pins on the map,
plus what the old GPS-only method would have given. **Use these for next flights** saves the winners.

| If you see | Try |
|---|---|
| One piece gets 2+ pins **on the same pass** | Raise *Frame-to-frame match distance* (0.5 → 0.8 m) or *Forget a piece after* (1.5 → 2.5 s) |
| One piece gets 2 pins **on neighbouring passes** | Raise *Same piece on another pass* (2 → 3 m) |
| Two close pieces become **one** pin | Lower *Same piece on another pass*; turn *Drop double boxes* off |
| Pins on nothing (false alarms) | Raise *Frames before…on the map* or *Confidence threshold* |
| Missed pieces you can see in the replay | Lower *Confidence threshold* |

Save good combinations as **presets** (Settings → Save current as preset…) to switch quickly between tests.
