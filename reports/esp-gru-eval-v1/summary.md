## Healthy runs with a persistent alarm (motor / IR / either)

| Method | Validation | Test |
|---|---|---|
| conv | 0 / 0 / 0 of 18 | 2 / 0 / 2 of 18 |
| conv-max | 0 / 0 / 0 of 18 | 1 / 0 / 1 of 18 |
| diff | 0 / 0 / 0 of 18 | 0 / 0 / 0 of 18 |
| stuck | 0 / 0 / 0 of 18 | 0 / 0 / 0 of 18 |
| gru | 0 / 0 / 0 of 18 | 1 / 1 / 2 of 18 |
| conv + gru | 0 / 0 / 0 of 18 | 3 / 1 / 4 of 18 |
| conv + diff | 0 / 0 / 0 of 18 | 2 / 0 / 2 of 18 |
| conv + stuck | 0 / 0 / 0 of 18 | 2 / 0 / 2 of 18 |
| conv + gru + stuck | 0 / 0 / 0 of 18 | 3 / 1 / 4 of 18 |
| conv-max + gru + stuck | 0 / 0 / 0 of 18 | 2 / 1 / 3 of 18 |
| conv + motor gru + stuck | 0 / 0 / 0 of 18 | 3 / 0 / 3 of 18 |
| conv + motor gru x2 + stuck | 0 / 0 / 0 of 18 | 2 / 0 / 2 of 18 |

## Fault detection by the primary model (runs detected; observable = runs with a monitored window after onset)

| Fault | observable | conv | conv-max | diff | stuck | gru | conv + gru | conv + diff | conv + stuck | conv + gru + stuck | conv-max + gru + stuck | conv + motor gru + stuck | conv + motor gru x2 + stuck |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| motor-weak-right-80 | 8/8 | 3/8 | 2/8 | 2/8 | 0/8 | 8/8 | 8/8 | 3/8 | 3/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-weak-right-70 | 8/8 | 6/8 | 5/8 | 1/8 | 0/8 | 8/8 | 8/8 | 6/8 | 6/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-weak-left-50 | 8/8 | 8/8 | 8/8 | 2/8 | 0/8 | 7/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-stall-left | 8/8 | 8/8 | 8/8 | 0/8 | 0/8 | 7/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-intermittent-right | 8/8 | 8/8 | 8/8 | 5/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| encoder-stuck-left | 8/8 | 8/8 | 8/8 | 0/8 | 0/8 | 6/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| encoder-noise-right | 8/8 | 8/8 | 8/8 | 8/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-stuck-center | 6/8 | 1/8 | 0/8 | 0/8 | 6/8 | 0/8 | 1/8 | 1/8 | 6/8 | 6/8 | 6/8 | 6/8 | 6/8 |
| ir-dead-left | 8/8 | 8/8 | 8/8 | 0/8 | 8/8 | 3/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-noise-right-50 | 8/8 | 0/8 | 0/8 | 0/8 | 0/8 | 1/8 | 1/8 | 0/8 | 0/8 | 1/8 | 1/8 | 0/8 | 0/8 |
| ir-noise-right-100 | 8/8 | 3/8 | 0/8 | 0/8 | 0/8 | 4/8 | 4/8 | 3/8 | 3/8 | 4/8 | 4/8 | 3/8 | 3/8 |
| ir-noise-right-200 | 8/8 | 8/8 | 8/8 | 1/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-noise-left-100 | 8/8 | 3/8 | 0/8 | 0/8 | 0/8 | 5/8 | 6/8 | 3/8 | 3/8 | 6/8 | 5/8 | 3/8 | 3/8 |
| ir-drift-center | 5/8 | 4/8 | 4/8 | 0/8 | 0/8 | 4/8 | 4/8 | 4/8 | 4/8 | 4/8 | 4/8 | 4/8 | 4/8 |
| ir-drift-right | 8/8 | 6/8 | 5/8 | 0/8 | 0/8 | 2/8 | 6/8 | 6/8 | 6/8 | 6/8 | 5/8 | 6/8 | 6/8 |

## Fault detection by either model (what stops the ESP)

| Fault | observable | conv | conv-max | diff | stuck | gru | conv + gru | conv + diff | conv + stuck | conv + gru + stuck | conv-max + gru + stuck | conv + motor gru + stuck | conv + motor gru x2 + stuck |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| motor-weak-right-80 | 8/8 | 3/8 | 2/8 | 2/8 | 0/8 | 8/8 | 8/8 | 3/8 | 3/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-weak-right-70 | 8/8 | 6/8 | 5/8 | 1/8 | 0/8 | 8/8 | 8/8 | 6/8 | 6/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-weak-left-50 | 8/8 | 8/8 | 8/8 | 2/8 | 0/8 | 7/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-stall-left | 8/8 | 8/8 | 8/8 | 0/8 | 0/8 | 7/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| motor-intermittent-right | 8/8 | 8/8 | 8/8 | 5/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| encoder-stuck-left | 8/8 | 8/8 | 8/8 | 0/8 | 0/8 | 6/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| encoder-noise-right | 8/8 | 8/8 | 8/8 | 8/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-stuck-center | 6/8 | 1/8 | 1/8 | 1/8 | 6/8 | 1/8 | 1/8 | 1/8 | 6/8 | 6/8 | 6/8 | 6/8 | 6/8 |
| ir-dead-left | 8/8 | 8/8 | 8/8 | 0/8 | 8/8 | 3/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-noise-right-50 | 8/8 | 5/8 | 4/8 | 4/8 | 0/8 | 2/8 | 5/8 | 5/8 | 5/8 | 5/8 | 4/8 | 5/8 | 5/8 |
| ir-noise-right-100 | 8/8 | 7/8 | 7/8 | 7/8 | 0/8 | 5/8 | 7/8 | 7/8 | 7/8 | 7/8 | 7/8 | 7/8 | 7/8 |
| ir-noise-right-200 | 8/8 | 8/8 | 8/8 | 8/8 | 0/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 | 8/8 |
| ir-noise-left-100 | 8/8 | 7/8 | 5/8 | 4/8 | 0/8 | 5/8 | 7/8 | 7/8 | 7/8 | 7/8 | 6/8 | 7/8 | 7/8 |
| ir-drift-center | 5/8 | 5/8 | 5/8 | 5/8 | 0/8 | 4/8 | 5/8 | 5/8 | 5/8 | 5/8 | 5/8 | 5/8 | 5/8 |
| ir-drift-right | 8/8 | 8/8 | 6/8 | 2/8 | 0/8 | 2/8 | 8/8 | 8/8 | 8/8 | 8/8 | 6/8 | 8/8 | 8/8 |

## Median detection delay, primary model (s)

| Fault | conv | conv-max | diff | stuck | gru | conv + gru | conv + diff | conv + stuck | conv + gru + stuck | conv-max + gru + stuck | conv + motor gru + stuck | conv + motor gru x2 + stuck |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| motor-weak-right-80 | 4.13 | 4.59 | 5.04 | - | 0.13 | 0.13 | 4.13 | 4.13 | 0.13 | 0.13 | 0.13 | 0.29 |
| motor-weak-right-70 | 1.78 | 0.74 | 9.18 | - | 0.10 | 0.10 | 1.78 | 1.78 | 0.10 | 0.10 | 0.10 | 0.18 |
| motor-weak-left-50 | 2.22 | 1.02 | 12.72 | - | 0.13 | 0.14 | 2.22 | 2.22 | 0.14 | 0.14 | 0.14 | 0.19 |
| motor-stall-left | 0.38 | 0.37 | - | - | 0.10 | 0.10 | 0.38 | 0.38 | 0.10 | 0.10 | 0.10 | 0.11 |
| motor-intermittent-right | 0.67 | 0.62 | 2.37 | - | 0.10 | 0.10 | 0.67 | 0.67 | 0.10 | 0.10 | 0.10 | 0.10 |
| encoder-stuck-left | 0.56 | 0.61 | - | - | 0.10 | 0.10 | 0.56 | 0.56 | 0.10 | 0.10 | 0.10 | 0.11 |
| encoder-noise-right | 1.06 | 0.91 | 0.96 | - | 0.14 | 0.14 | 0.96 | 1.06 | 0.14 | 0.14 | 0.14 | 0.18 |
| ir-stuck-center | 1.09 | - | - | 1.06 | - | 1.09 | 1.09 | 1.06 | 1.06 | 1.06 | 1.06 | 1.06 |
| ir-dead-left | 0.53 | 0.59 | - | 1.06 | 2.08 | 0.53 | 0.53 | 0.53 | 0.53 | 0.59 | 0.53 | 0.53 |
| ir-noise-right-50 | - | - | - | - | 13.22 | 13.22 | - | - | 13.22 | 13.22 | - | - |
| ir-noise-right-100 | 8.00 | - | - | - | 6.69 | 6.69 | 8.00 | 8.00 | 6.69 | 6.69 | 8.00 | 8.00 |
| ir-noise-right-200 | 2.16 | 3.65 | 4.06 | - | 2.82 | 2.16 | 2.16 | 2.16 | 2.16 | 2.78 | 2.16 | 2.16 |
| ir-noise-left-100 | 13.95 | - | - | - | 15.87 | 10.67 | 13.95 | 13.95 | 10.67 | 15.87 | 13.95 | 13.95 |
| ir-drift-center | 5.62 | 9.22 | - | - | 5.38 | 3.66 | 5.62 | 5.62 | 3.66 | 5.38 | 5.62 | 5.62 |
| ir-drift-right | 4.35 | 4.90 | - | - | 4.11 | 4.35 | 4.35 | 4.35 | 4.35 | 4.90 | 4.35 | 4.35 |

Fault runs with an alarm before onset: conv 0, conv-max 0, diff 0, stuck 0, gru 0, conv + gru 0, conv + diff 0, conv + stuck 0, conv + gru + stuck 0, conv-max + gru + stuck 0, conv + motor gru + stuck 0, conv + motor gru x2 + stuck 0
