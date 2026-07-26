# Regulatory Position → India

> **This is a project position statement, not legal advice.** Indian drone
> regulation has changed repeatedly since 2021 and may have changed again since
> the drafting assistant's knowledge cutoff of January 2026. Every rule stated
> here carries an explicit confidence level, and the low-confidence items
> **must be checked against current DGCA and Ministry of Civil Aviation text
> before submission**. A judge from Arduino, Qualcomm or Robu.in may know the
> current rules better than this document does.

---

## 1. Position

MonsoonReady is a student research and competition prototype. It has been flown
only in controlled test conditions, and demonstration flights dispense **inert
salt** rather than a licensed larvicide, so **no pesticide is applied at any
point**. Registration on the Digital Sky portal has not been completed, and
that is stated as an open gap rather than implying compliance that does not
exist. A deployed version would require the steps in §4, documented here so the
path from prototype to lawful operation is visible even though it has not been
walked.

---

## 2. Rules as understood

| Claim | Confidence | Detail |
|-------|-----------|--------|
| Weight categories | **Reasonably high; verify** | Under the Drone Rules, 2021, unmanned aircraft are categorised by all-up weight: Nano ≤ 250 g, Micro > 250 g to 2 kg, **Small > 2 kg to 25 kg**, and larger categories above. |
| MonsoonReady's category | **Reasonably high** | F550 with payload exceeds 2 kg → **Small**. Meaningfully more regulated than Micro, which is the practical consequence here. |
| Registration route | **High on existence, lower on detail** | Aircraft registration and unique identification number are issued through the Digital Sky platform |
| Airspace | **High on existence, lower on detail** | Operating zones governed by the airspace map's green, yellow and red zones |
| Pilot certification | **High on existence, lower on detail** | Remote pilot certification requirements exist and vary by category, via DGCA-approved training organisations |
| Aerial larvicide application | **Low. Must be researched from primary sources.** | A standard operating procedure for drone-based pesticide application exists on the agriculture side of government, and pesticide registration runs through the CIB&RC under the Insecticides Act, 1968. Whether public-health larviciding by drone is treated differently from agricultural spraying is **not established here**. |

---

## 3. Gaps

| Gap | Statement |
|-----|-----------|
| **Digital Sky registration** | Not complete. Self-registration attempts were blocked by the portal for this class of build, and no route through it was found as a two-person student team. No exemption is claimed, and no fault is attributed to the portal: the statement is simply that it could not be completed. |
| **Remote pilot certification** | Neither team member holds one |
| **Flight conditions** | Flights have been conducted as controlled prototype testing, in open areas away from people and property, within visual line of sight, at low altitude. Test site and conditions: TBD, to be described accurately rather than favourably. |
| **Larvicide registration status** | TBD. Whether the sourced Bti product is registered for this use in India is not established. A product photograph satisfies documentation needs, but says nothing about regulatory status. |

---

## 4. What lawful deployment would require

1. Aircraft registration and unique identification number.
2. Remote pilot certification for the operating pilot.
3. Operation confined to permitted airspace, with authorisation where required.
4. Confirmation that the larvicide is registered for the intended use, and
   compliance with whatever aerial-application procedure applies.
5. **Coordination with the municipal body responsible for vector control.**
6. Insurance and an operations manual appropriate to the category.

Item 5 is the one most easily forgotten and the most important in practice.
Larviciding public spaces is a municipal mandate, and an uncoordinated private
drone dropping granules into public water is a bad idea irrespective of
legality. This system is a tool for a vector-control programme, not a
replacement for one.

---

## 5. Why demonstrations use salt

Granular *Bti* is a biological larvicide that kills mosquito, blackfly and midge
larvae, widely used in public health programmes. Dispensing it in a
demonstration would convert a student flight test into a pesticide application,
invoking a body of law this project is neither qualified for nor licensed
under.

Demonstration flights therefore dispense **inert salt**, matched to Bti granules
for flow behaviour so the dispenser mechanism is tested honestly while the
material dropped is not a pesticide.

This is a design decision made early, not a technicality discovered late. The
dispenser is proven with an inert material **precisely so** that the
demonstration stays inside what the project is permitted to do.
