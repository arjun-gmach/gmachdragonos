# Turbine rotor: axial length reduction (Nov 2026)

GMACH Dragon 100 axial turbine rotor, redesigned in CFturbo (project `turbine2.3`, CFturbo 2025 R1.2) to cut wheel mass. The rotor's axial blade-row length goes from **14.45 mm to 8.27 mm**. The stator, annulus, blade count and design-point velocity triangles are unchanged.

## Summary

| | Baseline | Final |
|---|---|---|
| Rotor axial extension Δz | 14.45 mm (Δz/Δr 1.7307) | **8.27 mm (Δz/Δr 0.99)** |
| Blade axial length (LE → TE) | 13.87 mm | 7.94 mm |
| Wrap angle | — | **10°** |
| Throat area, total (Blade edges) | not checked | **959 mm²** (requirement ≈ 970 mm²) |
| Hub Zweifel (estimate, compressible) | ≈ 0.71 | ≈ 1.25 |
| Wheel mass, solid disc → profiled disc (estimate) | ≈ 365 g (solid) | **≈ 150 g** (profiled) |
| Polar moment of inertia (estimate) | ≈ 2.0 × 10⁻⁴ kg·m² | ≈ 0.9 × 10⁻⁴ kg·m² |

The final blade row passes the design mass flow through a correctly sized throat and keeps the hub loading within the usual 0.8–1.2 Zweifel range (hub at the upper limit). Most of the mass reduction comes from replacing the solid disc with a profiled disc. The shorter blade row adds the rest.

## Constraints

- **Stator already machined.** That fixes the annulus (hub Ø62.5 mm, shroud Ø79.2 mm, blade height 8.35 mm), the 17 vanes, and the rotor inlet swirl (free vortex, cu₁ = 505.9 m/s at mean, 581 m/s hub, 443 m/s shroud).
- **Rotor blade count fixed at 24** by manufacturing.
- **Material:** stainless steel, ρ = 7750 kg/m³.
- **Design point:** ṁ = 0.25 kg/s, 120 000 rpm, inlet pt = 3.15 bar, Tt = 700 °C, πts = 2.6.
- **Target:** turbine wheel mass close to the compressor wheel, ideally about 7 mm axial width.

## Final CFturbo settings (rotor, `Impeller_1`)

| Dialog | Setting | Value |
|---|---|---|
| Main dimensions | Hub / shroud Ø, tip clearance | 62.5 / 79.2 mm, 0.396 mm (unchanged) |
| Meridional contour | Axial extension ratio Δz/Δr | **0.99** (Δz = 8.267 mm) |
| Meridional contour | LE / TE position | 2 % / 98 % (z = 0.165 / 8.101 mm); LE/TE fixed on inlet/outlet: off |
| Blade properties | Number of blades | 24 |
| Blade properties | βB1 hub / mid / shroud | 131.0° / 108.3° / 79.6° (automatic) |
| Blade properties | βB2 hub / mid / shroud | 34.6° / 31.4° / 28.2° (automatic) |
| Blade properties | Slip δ hub / shroud | −5° / −5° (user defined) |
| Blade properties | Blockage thickness LE / TE | 0.8 / 0.8 mm |
| Blade mean lines | Wrap angle Δφ | **10°** (was 13.5°) |
| Blade mean lines | Overlap Δφ/t | 0.667 |
| Blade profiles | Thickness hub (LE → TE control points) | 0 / 1.42 / 1.008 / 0 mm |
| Blade profiles | Thickness shroud (LE → TE control points) | 0 / 1.411 / 0.976 / 0 mm |
| Blade profiles | PS control points hub / shroud | 0.801, 1.185, 0.248, 0.40 / 0.844, 1.165, 0.205, 0.40 mm |
| Blade profiles | Max. thickness | 1.427 mm at 29.2 % (hub), 1.425 mm at 27.8 % (shroud) |
| Blade profiles | LE / TE | rounded / rounded (TE ≈ 0.35–0.4 mm before round-off) |

Resulting flow and passage values (report 06):

| Quantity | Value |
|---|---|
| Throat per passage (Blade edges) | 39.94 mm² (o = 5.023 mm, h = 7.976 mm) |
| Throat total (× 24) | 959 mm² |
| Exit angle from o/s at mean (pitch 9.27 mm) | 32.8° (design 31.4°) |
| Outlet cu₂ / α₂ | 4.8 m/s / 89° |
| Outlet w₂ / relative Mach | 516 m/s / 0.905 |
| Rotor-relative inlet Mach | 0.333 |

## Iteration log

All reports are in [`cfturbo-reports/`](cfturbo-reports/). Throat values are from the **Blade edges** step, which includes the final thickness and edges.

| # | Report | Change | Throat per passage / total | Result |
|---|---|---|---|---|
| 0 | baseline ([screenshots](screenshots/baseline-14.45mm/)) | Δz 14.45 mm, Δz/Δr 1.7307 | not recorded | Hub Zweifel ≈ 0.71; solid disc ≈ 340 g |
| 1 | `01_dz7.0mm_wrap13.5` | Δz/Δr 0.836 → Δz 6.98 mm | 27.33 / 656 mm² | Throat about 33 % short of requirement, so the rotor would choke. Exit angle from o/s ≈ 22° against 31.4° design. Hub Zweifel ≈ 1.48. |
| 2 | `02_dz8.27mm_thinner-profile` | Δz/Δr 0.99 → Δz 8.27 mm. Max thickness 1.575 mm at 31.9 % → 1.33 mm at 25 %. Aft thickness 1.149 → 0.782 mm | 32.96 / 791 mm² | Hub Zweifel ≈ 1.25. Throat still about 18 % short. Thickness takes about 0.67 mm off the 4.82 mm mean-line opening. |
| 3 | `03_betaB2-closed_32-29-26` | βB2 set manually to 32 / 29 / 26° | 32.65 / 784 mm² | Wrong direction: more turning, relative exit Mach 0.979, cu₂ = −43 m/s. |
| 4 | `04_betaB2-opened_37.6-34.4-31.2` | βB2 set manually to 37.6 / 34.4 / 31.2° | 33.24 / 798 mm² | Throat almost insensitive to βB2 (+5.6° gave +0.6 mm²). cu₂ = +56 m/s, Euler work down about 10 % (200 against 223 kJ/kg). This showed the wrap angle was fixing the passage. |
| 5 | `05_wrap10deg` | Wrap 13.5° → 10°; βB2 back to automatic | 40.39 / 969 mm² | Throat matches requirement. cu₂ = 4.8 m/s. |
| 6 | `06_FINAL_profile-fixed` | PS aft control point raised from negative to 0.25 / 0.21 mm; TE control 0.4 mm; max thickness 1.43 mm at 29 % | **39.94 / 959 mm²** | Final. Smooth thickness distribution, no kinks; see [`screenshots/final/`](screenshots/final/). |

## Method and checks

**Throat requirement.** The rotor throat has to pass the design flow at the outlet relative velocity:

A_throat ≈ ṁ / (ρ₂ · w₂) = 0.25 / (0.50 × 516) ≈ **970 mm²**

At a relative exit Mach of 0.905 this is within about 1 % of the sonic area, so any shortfall in geometric throat chokes the rotor. A choked rotor raises the pressure between stator and rotor and pushes the compressor toward surge. The target was 38–40 mm² per passage (Blade edges → Throat cross-section area).

**Exit angle from the throat.** For a subsonic, throat-controlled cascade the effective exit angle is about sin⁻¹(o/s), measured from the circumferential direction. Pitch at mean radius is 2π × 35.425 / 24 = 9.27 mm.

**Blade loading (Zweifel).** Force-based form, with outlet axial velocity, corrected for compressibility at the outlet relative Mach number (factor about 1.2 at M ≈ 0.9):

Zw = 2 (s / b_x) · c_ax2 · Δw_u / w₂² ÷ [(p₀,rel − p₂) / (½ ρ₂ w₂²)]

| Span | 2·s·c_ax2·Δw_u / w₂² | Zweifel at b_x = 7.94 mm |
|---|---|---|
| Hub | 11.88 mm | ≈ 1.25 |
| Mean | 9.93 mm | ≈ 1.0 |
| Shroud | 8.43 mm | ≈ 0.83 |

The hub limits the axial length: highest turning (Δβ = 92.7°), highest Δw_u (581 m/s) and the least acceleration through the passage (w₂/w₁ = 1.82 against 2.91 at the shroud), all from the free-vortex swirl the stator delivers. With 24 blades, a 7 mm hub chord gives Zweifel ≈ 1.5, which leads to hub suction-side separation. The 8.27 mm extension is the practical minimum for this stator and blade count.

**Wrap angle.** The wrap angle is not recalculated when the axial extension changes. A 13.5° wrap over 7.94 mm of axial length forces an average blade angle of about 47° at the hub, so the blade does nearly all its turning just behind the leading edge and then runs straight. In that state the throat does not respond to βB2. Scaling the wrap roughly with axial length (13.5° × 7.94 / 14.3 ≈ 7.5°) brings it back into range; 10° hit the throat target.

## CFturbo notes from this redesign

- **β convention.** Blade and flow angles are measured from the circumferential direction. A larger βB2 means a more axial outlet: less turning and a larger throat.
- **Mean-line performance doesn't see chord.** Power (56.3 kW), η and velocity triangles are the same at any axial length, because η_tt is an input and the loss coefficients ζh and ζs are zero. Chord effects appear only in throat, loading and CFD.
- **The displayed Zweifel coefficient doesn't track the axial length.** Blade properties showed ψ = 1.15 at both 6.98 mm and 8.27 mm. Calculate Zweifel separately.
- **Wrap angle stays fixed.** After changing Δz, rescale the wrap in Blade mean lines.
- **Throat is the key passage check.** Read Blade edges → Throat cross-section area × blade count, and compare it with ṁ / (ρ₂ w₂).
- **Overlap below 1 is normal** for an axial turbine (final 0.667).
- **Rounded trailing edge.** With "Trailing edge rounded" on, the thickness list ends in 0; the physical TE is the rounded cap (≈ 0.35–0.4 mm here).

## Expected effect on performance

The changes against the 14.45 mm rotor pull in both directions:

| Effect | 14.45 mm | 8.27 mm | Direction |
|---|---|---|---|
| Hub loading (Zweifel) | ≈ 0.7 (under-loaded) | ≈ 1.25 (upper limit) | Small penalty at hub |
| Wetted area | larger | ≈ 45 % less | Gain |
| Chord / height (secondary losses) | ≈ 1.8 | ≈ 1.2 | Gain |
| Throat matched to flow | not checked | 959 / 970 mm² | Possible gain |

Estimated net difference: within about ±2 points of η_ts. A stage CFD run on both rotors (same stator, 120 000 rpm, πts = 2.6) will give the actual difference in mass flow, power and η_ts.

**Mechanical changes.** Gas bending stress at the blade root roughly doubles (same force per blade, about half the root chord). Centrifugal blade stress is unchanged. Blade natural frequencies shift and need checking against the stator wake excitation: 17 vanes × 2000 rev/s ≈ 34 kHz, plus harmonics.

## Mass estimate

Assumptions: stainless at 7750 kg/m³, Ø8 mm bore. Profiled disc has a rim the width of the blade row and about 4 mm deep (r 27.25–31.25 mm), a web tapering from about 6 mm at the boss to about 3 mm at the rim, and a boss at the bore.

| Wheel | Disc | Blades | Total |
|---|---|---|---|
| 14.45 mm, solid disc | ≈ 340 g | ≈ 25 g | ≈ 365 g |
| 14.45 mm, profiled disc | ≈ 180 g | ≈ 25 g | ≈ 205 g |
| 8.27 mm, solid disc | ≈ 195 g | ≈ 13 g | ≈ 210 g |
| **8.27 mm, profiled disc** | **≈ 135 g** | **≈ 13 g** | **≈ 150 g** |

The profiled 8.27 mm wheel is about 60 % lighter than the solid 14.45 mm wheel, with about 55 % less polar inertia. That roughly halves the spin-up torque for the compressed-air start.

## Open items

1. **Optional:** tick "Leading edge fixed on inlet" and "Trailing edge fixed on outlet" (blade 7.94 → 8.27 mm, hub Zweifel ≈ 1.2), then re-read the throat.
2. **Export** blades and hub surface only, not a solid.
3. **CAD disc:** rim 8.3 mm × 4 mm deep, tapered web, boss at bore, generous fillets at web-to-rim and web-to-boss.
4. **FEA** at 120 000 rpm and 120 % overspeed: bore hoop stress, web-to-rim fillet, blade root (centrifugal + gas bending), burst margin (average hoop stress against UTS), spinning modal check against 34 kHz. Inputs needed: rotor STEP, bore Ø and shaft attachment, exact stainless grade, temperature field (assumed ≈ 600 °C blades/rim, ≈ 300 °C bore until measured). Creep is likely the limit above 600 °C for stainless.
5. **Stator–rotor axial gap** is 0.74 mm (stator TE z = −0.577 mm, rotor LE z = 0.165 mm). Confirm it against bearing axial float and thermal growth.
6. **Stage CFD** comparison of the 8.27 mm and 14.45 mm rotors.
7. **CFturbo Euler/enthalpy power discrepancy** is unchanged by this redesign. Rotor Tt drops 700 → 602.7 °C (≈ 98 K), against Euler work of 223 kJ/kg.

## Files

| Path | Contents |
|---|---|
| `cfturbo-reports/01_…` to `06_FINAL_…` | CFturbo stage reports for each iteration (tab-separated text export) |
| `screenshots/baseline-14.45mm/` | Baseline rotor: Main dimensions (Setup, Parameters, Dimensions), design-point values, meridional contour at Δz = 14.45 mm, Blade properties (setup, angles) |
| `screenshots/final/` | Final Blade profiles dialog: thickness distribution, 3D preview, meridional thickness |
