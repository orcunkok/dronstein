# Office realism to-do

The layout and sizes match the photos (`office_params.toml`); the look does not. The scene uses flat colours, raw boxes and
point lights, while the kit's own examples get their look from textures, bevels and area lights. Apply these in order
(biggest visual payoff first), rebuild with `build_office.py` + `import_office.py`, and compare renders with the photos
in `lucky_office/` (IMG_3077 is the best overall reference).

Kit references (path changes when Lucky restarts: `ls -d /tmp/.mount_Lucky*/Data/Authoring`):
- `rosewood_textures.py`: generates seamless PBR textures (`*_BaseColor.png`, `*_Normal.png`, `*_MetallicRoughness.png`).
- `rosewood_objects.py`, `assemble_rosewood_scene.py`: textured objects and how the texture files ship with the package.
- `python/examples/living_room_daylight.py`: area lights, daylight, `UVProject`/`UVTransform`, bevels.
- `python/docs/recipes.md`: cloth (`ClothDrape`), cushions, rounded shells, material channels.

## 1. Textures (PBR), generated in Python like `rosewood_textures.py`
- [ ] Tilt-up concrete walls: off-white with form-panel seams (horizontal lines ~1.2 m apart), anchor holes, stains.
- [ ] Roof deck: white-painted timber planks (~0.15 m boards) running across the purlins.
- [ ] Epoxy floor: mid-grey, slightly glossy, scuffs, saw-cut slab joints; white test mat with painted rings.
- [ ] Stage curtain: velour fabric (black east face, white west face) with a weave normal map.
- [ ] Steel: brushed galvanised for cable rails and posts; matte charcoal paint for the U stair.
- [ ] Kitchen: white shaker cabinets (light grain), black stone counter, dark tile backsplash.
- [ ] Map them with `UVProject`/`UVTransform` (box projection, metre scale) and ship the PNGs as package resources
      (`PackageClient.publish(..., resources=[...])`, see `assemble_rosewood_scene.py`).

## 2. Lighting
- [ ] Replace the 9 point lights (`OfficeLight_*`, `OfficeFill_*`, made by `office_scene.py`) with rectangular area lights
      at the LED high-bay positions (`build_shell`), pointing down.
- [ ] Daylight: an area light per roof skylight and soft light through the clerestory and south windows.
- [ ] Tune exposure and intensity for the bright, even look of IMG_3077, with no glare spots near the ridge.

## 3. Geometry detail
- [ ] Bevel edges (`BevelEdge`) on walls, pilasters, trusses, cabinets, desks and the box/mezzanine.
- [ ] Trusses and purlins with real steel profiles (angles and channels) instead of square bars.
- [ ] Windows with frame depth and recessed glass; the roller gate with a ribbed curtain.
- [ ] Curtain: finer pleats, smooth shading, a slight drape at the hem (`ClothDrape`, few steps).
- [ ] More kit library objects where they exist (shelving, boxes, lamps) instead of plain boxes.

## Notes
- Keep the sizes in `office_params.toml`; these changes are about the look only.
- Verify each step with renders from `shots.py`-style review views and a Play check (`X500Flight: start` after the build).
