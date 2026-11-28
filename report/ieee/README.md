# IEEE report

`main.tex` is the project paper in IEEE conference format (`IEEEtran`), with
one section per grading criterion: System Design & Architecture,
Implementation, Functionalities, Testing, Performance & Evaluation Results,
and GitHub Repository. Figures are in `figures/`; the architecture diagram is
drawn in TikZ inside `main.tex`.

## Build the PDF

**Overleaf (no install):** New Project -> Upload Project -> upload a zip of
this folder (`main.tex` and `figures/`), then Recompile. `IEEEtran` and every
package used are built in.

**Locally** (TeX Live or MiKTeX):

```powershell
cd report\ieee
pdflatex main.tex
pdflatex main.tex
```

## Before submitting

- Replace the author block placeholders (names, department, university,
  emails) near the top of `main.tex`.
- All numbers come from `REPORT.md` and `reports/` as of commit `622e6ea`, plus
  the resource table from `reports/esp-resources-v1`. If
  models or results change, update the tables to match.
