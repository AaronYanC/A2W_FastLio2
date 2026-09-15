# Third-party notices

## Scan Context

- Source: `https://github.com/engcang/scancontext_tro`
- Revision: `c8ef5b496a159cdfd7fa4761121178f25cd0a6bb`
- License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0
- Copyright: KAIST and Naver Labs

Only the ROS-independent polar descriptor core is included. See
`third_party/scancontext_tro/UPSTREAM.md` for the adaptation record.

## Quatro

- Source: `https://github.com/engcang/Quatro`
- Revision: `d27109bd6a1798e9cf2e0c2a4daac6af16e7bc23`
- License file: GNU General Public License v3.0
- Additional upstream metadata mentions CC BY-NC-SA 4.0; treat the combination conservatively

Only the FPFH, matcher, and Quatro module core is included. ROS 1/catkin glue is excluded. See
`third_party/Quatro/UPSTREAM.md`.

## Nano-GICP

- Source: `https://github.com/engcang/nano_gicp`
- Revision: `b21e79edcceb7c6e86ec0dfec90f451b796b683a`
- License: MIT; individual retained source notices also contain BSD-3-Clause terms

Only the registration core is included. See `third_party/nano_gicp/UPSTREAM.md`.

## TEASER++

- Source: `https://github.com/MIT-SPARK/TEASER-plusplus`
- Revision: `974574c2fe8d8523e9f7d7be0500c427ba1d1f9a`
- License: MIT

The pinned submodule provides the robust registration solver required by Quatro. The build uses
only solver/graph sources and does not build its I/O, examples, tests, Python, or MATLAB targets.

## Parallel Maximum Clique (PMC)

- Source: `https://github.com/jingnanshi/pmc`
- Revision: `a2dfd612a501bca83c47206255dbbff619481f97`
- License: GNU General Public License v3.0 or later

The pinned submodule is a private transitive dependency of TEASER++ registration. CLI and test
programs are not built.
