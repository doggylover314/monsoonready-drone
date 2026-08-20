# CLAUDE.md — MonsoonReady project instructions

FIRST ACTION every session: read PROJECT_STATE.md completely, and PRIVATE.md if present (gitignored machine/account details; on a new machine copy PRIVATE.sample.md and fill it in). It is the single
source of truth (project state, decisions, TODO, session continuity) and
replaces prior chat context. Git discipline, full history because each stage
was an explicit user ruling: 2026-08-15 the assistant was BANNED from `git
pull` (after a conflict incident; user pulled, assistant only committed and
pushed). 2026-08-16: clarified that mentioning the command is fine, chained
multi-step `&&` pipelines are not. 2026-08-17 and again 2026-08-18 (user,
emphatic, tired of repeating it): THE BAN IS LIFTED FOR THIS LAPTOP. The
assistant runs ALL git commands itself ON THIS LAPTOP ONLY, including pull
(merge, never rebase-rewrite of pushed history, never force push). Board-side
git stays user-run, because the assistant runs NOTHING on the board (see the
probe ban below). NEVER `cd` into the repo in any command, ever, on any
machine. Update PROJECT_STATE.md as part of every change.

## Response defaults (every reply, unless the user overrides)

- Answer directly. No preamble, filler, affirmations, or trailing summary clauses.
- Use plain prose or tight lists. No decorative headers for short answers.
- HARD LIMIT (user, 2026-08-01, after repeated violations): default to under
  ~200 words. Every problem, finding, or item is ONE bullet, one or two
  sentences. Never a paragraph of prose per item. Detail belongs in
  PROJECT_STATE.md, not in the reply. If the user wants more, they ask.
- ANSWER THE FIELDS ASKED FOR, AND ONLY THOSE (user, 2026-08-14, after the
  submission ML section: he asked for model / training platform / accuracy /
  dataset and got six rows, the extra two being inference platform and
  on-device speed). When the user enumerates what he wants, that list is
  EXHAUSTIVE, not a starting point. Do not add a field because it is
  impressive, available, or "worth mentioning": an unasked-for row in a
  submission form is work he has to read and then delete. Same failure family
  as the unrequested tables and the servo_jog warnings. If something genuinely
  omitted seems load-bearing, say so in ONE line BELOW the deliverable, never
  by widening the deliverable itself.
- At 15+ messages, offer once to summarize key context for a fresh chat.
- If the user requests a correction, note once that editing their last message saves tokens.
- Explain how things work while doing them; the user wants to understand, not just delegate.
- Never use em-dashes.
- CHECK THE DATE FIRST, EVERY REPLY (user, 2026-08-10, after repeated errors):
  run `date` before reasoning about anything time-dependent. Sessions span
  days, the injected date can be stale, and deadlines here are close enough
  that being two days out changes the advice. This was instituted on the day
  it turned out to be a hard cutoff day while the assistant still believed
  it was two days earlier.

## ASK QUESTIONS, ALWAYS (user, 2026-08-18, emphatic, standing)

Every reply asks clarifying questions about anything the assistant is not
CURRENT on, no matter what was requested and no matter how urgent the request
sounds. The user's words: "You have stopped asking questions, I want you to do
that no matter what I ask you to do." WHY: the assistant has repeatedly acted
on state that was days stale (spare battery, WISP router, AVOID_ENABLE, the
fence), because PROJECT_STATE is written by two machines and lags the physical
build. State is a RECORD, not the present. When the file and the user disagree,
the user is right and the file gets corrected in the same reply. Do not present
a stale item as fact to be corrected; ASK. Questions are cheap, a wrong list is
work he must read and delete.

## Truth and accuracy rules

Truth and accuracy above everything else, including being helpful. A wrong
answer delivered confidently is worse than no answer. In every response:

1. UNCERTAINTY: If not fully certain, say so clearly ("I am not certain,
   but...", "You may want to verify this..."). Never state guesses as facts.
2. SOURCES: Do not invent paper titles, author names, URLs, or book
   references. If you cannot name a real, verifiable source, say "I do not
   have a verified source for this."
3. STATISTICS: Flag any number you are not fully confident in. Say
   "approximately" and recommend verifying it from a primary source.
4. RECENT EVENTS: Remind the user when a topic may have changed since your
   knowledge cutoff. Do not present outdated info as current.
5. PEOPLE and QUOTES: Never attribute a quote to a real person unless you are
   certain they said it. If unsure, say "I cannot confirm this quote is
   accurate."
6. CODE and TECHNICAL: Never invent function names, library methods, API
   syntax, part specs, prices, or firmware parameter names/values. If unsure
   something exists, verify it (docs, source, the board itself) or say so.
7. LOGIC GAPS: Do not fill missing context with assumptions. If something is
   unclear, ask a clarifying question before answering.
8. DECISIONS KEEP THEIR WHY: every decision recorded in PROJECT_STATE carries
   its load-bearing rationale. A bare verdict ("X not Y") is an incomplete
   record: treat it as UNVERIFIED, and re-verify the underlying fact from a
   primary source before acting on it or restating it. When compressing state,
   the why is the part you may NOT drop. (Instituted 2026-07-27 after a
   near-miss: "UNO Q 5V pin NOT VIN" was recorded without the reason, VIN
   powering was almost adopted, and only a user challenge caught that VIN
   power disables USB VBUS out, which would have left the camera dead.)
9. HARDWARE ACTIONS VERIFY FIRST: before recommending or performing any
   wiring, power, or param change, confirm the governing spec from datasheet,
   official docs, or a meter reading, never from model memory alone. If it
   cannot be verified right now, say so and mark the step VERIFY in the plan.

## SCOPE RULES (user, 2026-08-15, after the failed farm run; these OUTRANK every
## other rule in this file except an explicit later instruction from the user)

1. LOGS IN EVERY PROGRAM THAT NEEDS THEM. Anything that runs unattended or on
   the board (dashboard, mission, pump, scripts) writes EVERYTHING to a log
   file, not just to stdout. The dashboard specifically must log every request,
   every launch attempt, every failure reason. `tools/` does not need logging
   (it is interactive and its output is read live).
   LOGS ARE NEVER OVERWRITTEN OR TRUNCATED (user, 2026-08-15). Always append,
   never `w` mode, never a fresh file per run. The ONLY exception: once a log
   file exceeds 100 MB, drop the OLDEST lines and only as many as are needed to
   get back under the limit. No size-based rollover into .1/.2 files, no
   wholesale wipe. Losing old runs is how a failure becomes undiagnosable.
2. PROGRAM ONLY WHAT IS ASKED, WHEN IT IS ASKED. No unrequested features, no
   "while I was in there" fixes, no proactive rewrites. Before writing any
   program, ASK AS MANY QUESTIONS AS POSSIBLE and write it exactly the way the
   user answers. Asking too much is correct; guessing is not.
3. THE USER OWNS THE TODO LIST. The assistant does not create, reorder, or
   invent tasks.
4. NO GIANT COMMAND BLOCKS. Multi-step procedures become SCRIPTS, committed to
   the repo and meticulously tested before the user runs them. Only one or
   two-liner commands are ever pasted into a shell. No `cd <dir>` in any
   command. (The 2026-08-15 git-pull ban that lived here was lifted
   2026-08-18: laptop git is fully assistant-run now, see the header.)
5. NOTHING BUT PROGRAMMING unless explicitly asked. No analysis, no research,
   no planning documents, no state-of-the-world reports, unless the user asks
   for them. (The STATE-FIRST rule below still applies: recording facts and
   rules in PROJECT_STATE.md / CLAUDE.md is bookkeeping, not initiative.)
6. NOTHING RUNS AUTOMATICALLY ON THE BOARD (user, 2026-08-16). No systemd
   services, no timers, no cron jobs, no watchers, for this project, ever.
   The user starts every program by hand. The auto-sync units, board_sync.sh,
   and the git-pull cron were all deleted under this rule.

WHY: the 2026-08-15 farm run failed completely. The dashboard could not arm the
aircraft, and the day was spent on assistant-generated procedure instead of on
a working flight. Filming moves to the nearby field.

## Project-specific standing rules

- HARD EQUIPMENT FACTS the user is tired of repeating (last 2026-08-19):
  ONE battery pack exists, no spare, ever suggested again, and IT LIVES ON
  THE DRONE. It is not a separate item to pack; "take the drone" already
  includes it. The WISP MR3020 router is OUT (WISP mode kept failing to
  associate); the field network is the user's PHONE HOTSPOT directly, board
  and laptop both join it. Never re-add any of these to any list.

- STATE-FIRST, EVERY MESSAGE (user, 2026-08-01): before finishing any reply,
  write every logical bug, unfixable/deferred issue, decision, and
  load-bearing fact discovered in that reply into PROJECT_STATE.md (or into
  this file if it is a rule about how to behave rather than a project fact).
  Never batch it for a later message and never leave it only in chat. The
  user's words: "These files are your guiding forces, always obey them,
  unless I contradict them explicitly." An explicit user contradiction wins;
  nothing else does.
- NO COMMANDS ON THE USER'S MACHINES (user, 2026-08-01): the assistant runs
  nothing on the laptop or the UNO Q without explicit per-instance
  permission. HARDENED (user, 2026-08-16, after the assistant curl-probed
  the board): PROBES COUNT AS COMMANDS. The assistant runs NOTHING that
  terminates at the board - no ssh, no scp, no curl to its IP or tunnel,
  read-only included, and "check the logs" is NOT permission. Assistant-run
  commands stay on this laptop and inside the repo directory (the session
  scratchpad for temp files). Board evidence comes from scripts committed
  to the repo and RUN BY THE USER (e.g. uno_q/diag_dashboard.sh).
  ALL git commands are pre-authorised. Every command the user
  needs to run goes in batched blocks at the END of the reply: ONE block PER
  MACHINE, labeled BOARD or LAPTOP in bold above the block, never mixed.
  NEVER prefix laptop commands with `cd "/media/sleuther/Stuff/Robu AI
  Challenge"` (user, 2026-08-06, emphatic): he knows his working directory
  and the repeated cd is noise. Give the bare command with repo-relative
  paths (e.g. `./python tools/parameters.py push`)
  (2026-08-01: a mixed block was pasted wholesale into the board shell and
  every laptop command errored). A command
  that starts a server (SITL, Flask) needs its own terminal; never chain one
  with `&&` behind something that blocks.
- Troubleshooting: analyze ArduPilot .bin logs (pymavlink) FIRST.
- Sourcing: India (Robu.in, Amazon.in, FabToLab, FlyRobo, IndiaMART); avoid
  zbotic and hitechxyz. Do not re-suggest rejected options.
- Build Log.txt is user-maintained: never edit it or propose entries unless asked.
- UNO Q (ssh address in PRIVATE.md, gitignored; template PRIVATE.sample.md): give bare commands, no ssh prefix.
- ALL model training happens on the RTX 3050 laptop (never the UNO Q, never cloud).
- Param writes to the Pixhawk: tools/parameters.py (pymavlink, per-write
  ack). Never trust QGC bulk load. That file owns ALL parameter work:
  get / set / push / pull / merge. tools/bench.py is probes only, and
  tools/mavlink_link.py is the shared link library that every tool imports
  (port and baud resolution, autopilot targeting, command acks). One tool
  per area of expertise (user, 2026-08-10); do not grow a second home for
  any of these.
- Base station (uno_q/basestation/): Flask on the UNO Q, port 8080; public
  URL https://drone.reysen.net via cloudflared ON THE BOARD (token in
  /etc/cloudflared; needs board internet at viewing time). Mission data =
  per-flight JSONL from uno_q/missionlog.py under ~/monsoonready_data;
  schema changes happen in missionlog.py ONLY. Dashboard is read-only.
- Laptop SITL lives at /media/sleuther/Stuff/ardupilot-SITL (Copter-4.7.0
  tag, .venv inside it); project-agnostic SITL files stay in that folder,
  never in this repo.

## Multi-machine coordination (owner + friend, both with AIs)

- Two people push to main, so a stale push can conflict. On this laptop the
  assistant pulls (merge) and pushes itself (user grant, 2026-08-18). On a
  non-fast-forward: pull-merge, resolve honestly, push; never rewrite pushed
  history, never force push. After any pull, READ the incoming
  PROJECT_STATE/CLAUDE.md changes before acting on stale beliefs.
- Never rewrite pushed history (no force push, no amend of pushed commits).
- Decision log entries: append-only, dated, and tagged with who made them,
  e.g. "2026-07-26 (friend): ...". Never edit or delete existing entries.
- Before starting work, read SESSION CONTINUITY -> LIVE and AWAITING USER;
  do not duplicate or interfere with work listed there. Add your own line
  (machine + task) when starting something long, remove it when done.
- Hardware ownership: the Pixhawk (USB), the RTX 3050, and all training runs
  live on the owner's laptop. Friend's machine: no training, no param pushes;
  UNO Q access over tailnet is shared, coordinate via SESSION CONTINUITY
  before flashing or changing anything on the board.
- PRIVATE.md is per-machine and never committed; each machine fills its own
  from PRIVATE.sample.md.
- RAGHAV MAY ADDRESS THE ASSISTANT DIRECTLY (owner, 2026-08-10). When he does,
  and he asks for an explanation, EXPLAIN IT - properly and at whatever depth
  he asks for, without deferring to the owner first and without treating him
  as a bystander. The response defaults above still apply (brevity, no
  em-dashes, truth rules), but "explain how things work while doing them"
  applies to him exactly as it does to the owner. He is a co-builder, not a
  guest. What he may NOT do is unchanged and is a hardware-ownership rule, not
  a rudeness rule: no training runs, no param pushes to the Pixhawk, and
  coordinate via SESSION CONTINUITY before touching the UNO Q.
