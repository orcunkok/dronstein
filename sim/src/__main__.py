"""sim/.venv/bin/python sim/src [--lucky [HOST]] [--model sih|dsb-sim] [--speed 1|0] [--home lat,lon,alt] [--instance N]
[--log truth.csv]"""
import argparse
import os

import physics
import px4
from lucky import Lucky

ap = argparse.ArgumentParser(prog="sim/src", description="Headless X500 physics for PX4 SITL over MAVLink HIL.")
ap.add_argument("--model", choices=["sih", "dsb-sim"], default="dsb-sim", help="force model (default dsb-sim)")
ap.add_argument("--lucky", nargs="?", const="127.0.0.1", metavar="HOST",
                help="fly the drone in Lucky's MuJoCo (its X500Flight script), listening on HOST (default 127.0.0.1)")
ap.add_argument("--speed", type=float, default=1.0, help="1 = real time, 0 = as fast as PX4's lockstep allows")
ap.add_argument("--home", help="lat,lon,alt (deg, deg, m AMSL); default from params.toml")
ap.add_argument("--instance", type=int, default=0, help="listen on tcp 4560 + N")
ap.add_argument("--log", help="write the true state every step to this CSV")
ap.add_argument("--params", default=os.path.join(os.path.dirname(__file__), "..", "params.toml"))
args = ap.parse_args()

prm = physics.load(args.params)
if args.home:
    lat, lon, alt = (float(x) for x in args.home.split(","))
    prm["home"].update(lat=lat, lon=lon, alt=alt)
    prm["g"] = float(physics.gravity(lat))
try:
    px4.serve(prm, args.model, 4560 + args.instance, args.speed, args.log, Lucky(args.lucky, prm) if args.lucky else None)
except KeyboardInterrupt:
    pass
