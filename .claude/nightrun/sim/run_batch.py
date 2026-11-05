"""Success-rate table. Usage: python run_batch.py [N_nominal] [N_ring]
Categories: nominal (random ball in [-1.6,-0.5]x[-0.8,0.8], random start pose/heading, random friction 0.5-1.5, mass +-50%),
ring (ball 1.1-1.6 m from the bin on a random side), corners (friction {0.5,1.5} x mass {0.5,1.5}x)."""
import sys, json, time, math
import numpy as np
from pick_proto import *

def scene(kind, i):
    rng = np.random.default_rng(1000 + i)
    s = random_scene(1000 + i)
    if kind == 'nominal':
        s['ball_pos'] = (rng.uniform(-1.6, -0.5), rng.uniform(-0.8, 0.8))
    elif kind == 'ring':
        r, th = rng.uniform(1.1, 1.6), rng.uniform(0, 2 * math.pi)
        s['ball_pos'] = (-3.0 + r * math.cos(th), r * math.sin(th))
    elif kind == 'corners':
        s['ball_fric'] = [0.5, 1.5][i % 2]; s['ball_mass'] = 0.0459 * [0.5, 1.5][(i // 2) % 2]
    return s

def run(kind, i, **kw):
    s = scene(kind, i)
    sim = Sim(seed=i, ball_pos=s['ball_pos'], bin_pos=s['bin_pos'], start_yaw=s['start_yaw'], start_xy=s['start_xy'],
              ball_fric=s['ball_fric'], ball_mass=s['ball_mass'], dist_force=kw.get('dist_force', 0.0))
    m = Pick(sim).run()
    m['a_ok'] = bool(m.get('ball_moved_cm_at_ground', 99) < 1.0 and m.get('ball_moved_cm', 99) < 1.0 and not m.get('drone_ground_contact'))
    m['b_ok'] = bool(m.get('grasp_detected') and m['max_ball_tool_cm'] < 3.0 and m.get('lift_ball_z', 0) > 0.5)
    m['c_ok'] = bool(m['ok'])
    m['clear_ok'] = bool(m['min_clear_above_rim_cm'] is None or m['min_clear_above_rim_cm'] >= 10.0)
    m['scene'] = {k: (list(map(float, v)) if hasattr(v, '__len__') else float(v)) for k, v in s.items()}
    return m

if __name__ == '__main__':
    nn = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    nr = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    allres = {}
    for kind, n in (('nominal', nn), ('ring', nr), ('corners', 8)):
        rs = []; t0 = time.time()
        for i in range(n):
            m = run(kind, i); rs.append(m)
            if not m['ok'] or not m['clear_ok']:
                print(f"  {kind} #{i}: ok={m['ok']} fail={m['fail']} a={m['a_ok']} b={m['b_ok']} clear={m['min_clear_above_rim_cm']} moved={m.get('ball_moved_cm_at_ground')} binmv={m['bin_moved_cm']:.2f} inbin={m['ball_in_bin']} scene={m['scene']}")
        f = lambda k: sum(r[k] for r in rs)
        print(f"{kind:8s} n={n:3d}  (a) approach {f('a_ok')}/{n}   (b) grasp+lift+carry {f('b_ok')}/{n}   (c) in bin & bin unmoved {f('c_ok')}/{n}   clearance>=10cm {f('clear_ok')}/{n}   "
              f"max tilt {max(r['max_tilt_deg'] for r in rs):.1f} deg  median T {np.median([r['t_total'] for r in rs]):.0f}s  [{time.time()-t0:.0f}s wall]", flush=True)
        allres[kind] = rs
    json.dump(allres, open('results.json', 'w'), default=float, indent=1)
