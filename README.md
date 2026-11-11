# SHEFF DoF Manager (`sheff-dof-manager`)

`sheff-dof-manager` enumerates, updates, and maps nodal and element degrees of freedom (DoFs) to global linear system indices in the **Scalable Hierarchical Enrichment Finite-element Framework (SHEFF)**, seamlessly handling both standard continuous-Galerkin fields and dynamically evolving hierarchical XFEM enrichments.

## Capabilities

- **Continuous-Galerkin & Enriched DoF Enumeration (`DofManager.h`, `detail/BasalFieldCompatibilityMap.h`)**:
  - Automatically matches compatible basal fields across shared nodes, edges, and faces of adjacent elements so standard and multi-level Heaviside-enriched fields maintain $C^0$ continuity where unbroken and independent DoFs across crack surfaces.
- **Local-to-Global Mapping (`DofMap.h`, `ElementScatterMap.h`, `NodeScatterMap.h`, `DofRanges.h`)**:
  - Provides fast element-to-global and node-to-global scatter/gather maps (`ElementScatterMap`, `NodeScatterMap`) for matrix assembly, solution extraction, and boundary condition enforcement, including free-DoF tracking for periodic boundary conditions.
- **Distributed & Bandwidth-Friendly Ordering (`detail/DofSorting.h`)**:
  - Assigns contiguous rank-owned DoF ranges for MPI-distributed solves and supports dynamic re-enumeration as new XFEM cracks initiate and propagate during an analysis.

## Project Structure

- `include/dof_manager/`: Public headers (`DofManager.h`, `DofMap.h`, `ElementScatterMap.h`, `NodeScatterMap.h`, `DofRanges.h`) and `detail/` helpers.
- `source/`: DoF compatibility graph construction, numbering, sorting, and scatter-map builders.
- `unit_tests/`: Unit tests verifying DoF numbering on unenriched, cracked, intersecting-crack, and partitioned meshes.
- **CMake Package / Target**: `DofManager` / `DofManager::dof_manager` (static library).

## Dependencies

- **Build System**: [`sheff-build-tools`](https://github.com/afrl/sheff-build-tools) (`SHEFF_TOOLS`), CMake 3.17+, C++17/C++20 compiler.
- **First-Party SHEFF Modules**: [`sheff-finite-element`](https://github.com/afrl/sheff-finite-element) (`FiniteElement::finite_element`).

## Building

When building as part of [SPAWC](https://github.com/afrl/spawc), `sheff-dof-manager` is configured and built automatically via `make.py`.

To build and test standalone (with `SHEFF_TOOLS` configured):

```bash
python3 make.py --config --build --make-tests --run-tests
```

## Public Release Information

Distribution Statement A. Approved for public release: distribution is unlimited. Case #: AFRL-2026-4513
