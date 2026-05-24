# lacam3 with FoV (Field of View) support

This repo is the c code implementation of LaCAM* with FoV support, PIBT with FoV support, and the PPfPP algorithms from the k-privacy preserving MAPF solver.

Read more about the algorithms in the published paper *Privacy Preserving Multi-Agent Path Finding* at:
* Extended abstract published at AAMAS 2026:
    https://ifaamas.org/Proceedings/aamas2026/pdfs/JWHZ7620.pdf
* Full paper published at arXiv:
    https://arxiv.org/abs/2605.14119


This is a fork of the lacam3 code from https://github.com/Kei18/lacam3.

If you plan to use this code or the code at https://github.com/SPL-BGU/K-Privacy-Solver-Python, please cite our papers from above, and also see citation instructions from the lacam3 repo.


## Building

All you need is [CMake](https://cmake.org/) (≥v3.16).
The code is written in C++(17).

First, clone this repo with submodules.

```sh
git clone --recursive https://github.com/SPL-BGU/lacam3_with_FoV_support.git && cd lacam3_with_FoV_support
```

Then, build the project.

```sh
cmake -B build && make -C build
```

## Usage

Please look at the repo https://github.com/SPL-BGU/K-Privacy-Solver-Python which is the main repository for running this code.

## Contributing

### install pre-commit for formatting

```sh
pre-commit install
```

### make a pull request with your suggested changes

If you plan to contribute to this repo, please create a pull request and contact the author of this repo.
