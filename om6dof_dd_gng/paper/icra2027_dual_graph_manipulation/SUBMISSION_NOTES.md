# ICRA 2027 submission notes

Checked 2026-09-14 against the official technical-paper call:

- [Official ICRA 2027 call for papers](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)
- [PaperCept LaTeX support](https://ras.papercept.net/conferences/support/tex.php)
- [RAS double-anonymous rules](https://www.ieee-ras.org/publications/rules-for-the-double-anonymous-review-process/)
- [RAS generative-AI guidelines](https://www.ieee-ras.org/publications/guidelines-for-generative-ai-usage/)

The official CFP lists **September 15, 2026, 23:59 PST** as the paper deadline.
This reproduces the organizer's timezone label; confirm the portal cutoff rather
than interpreting PST/PDT locally. It specifies **eight pages total**, including
figures, acknowledgments and references, in double-column PDF format with
double-anonymous review. The current document uses anonymous authors and avoids
custom robot names, local usernames, and project paths in the rendered PDF.

The official site contains some inherited 2025 text on other pages and lower
sections. Those stale final-submission instructions were not used to set the
2027 manuscript format. The source of the template class is PaperCept's
`ieeeconf` distribution, already available in the project.

The manuscript includes an AI disclosure identifying OpenAI Codex (GPT-6) and
the affected writing/analysis/figure work, in accordance with the linked IEEE-RAS
guidelines. It does not claim that human authors have already approved this draft.
Author identities, affiliations, funding, and submission identifiers were not
invented. No submission, public upload, or communication with coauthors occurred.

## Research readiness

The document is a complete preliminary manuscript, not a finished empirical
manipulation study. Its new quantitative evidence is an offline reanalysis of a
recorded workspace scan and its frame conventions. A controller test count is
not a sample size for grasp success. Console screenshots are not an independent
dataset of successful manipulation trials.

The older draft at `../icra2027_reachability/` already contains dual topology,
configuration retention, and lazy collision-validation ideas. The new draft
does not claim those ideas as individually novel or reassign its 21,600 queries
to the new semantic pipeline. Before submitting either manuscript, assess their
conceptual and textual overlap and whether the new study establishes sufficient
additional contribution. Publication/submission status of the older paper is
not known from this workspace and has not been assumed.

Mechanical checks in `verification.json` cover the locally compiled PDF. They
do not replace PaperPlaza PDF checks, scientific review, or the missing studies
in `EVALUATION_PLAN.md`. Inspect the final source bundle before sharing it:
audit notes retain internal repository paths for traceability.
