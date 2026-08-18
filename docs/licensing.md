# License selection before public release

The repository scaffold intentionally **does not choose legal terms on behalf of the project owner**. Replace the root `LICENSE` placeholder before calling the repository open source.

A practical option to consider for the software and documentation is **Apache License 2.0**, because it is permissive and includes an explicit patent grant. MIT is a simpler permissive alternative. If this repository later contains custom PCB/CAD hardware design files, consider whether those files should use a hardware-specific license (for example a CERN Open Hardware Licence variant) rather than inheriting a software license accidentally.

This is project-planning guidance, not legal advice. Whatever is selected should be stated clearly in the root README and, if directory-specific licenses are used, in those directories.

Do not assume that a project license grants permission to redistribute third-party firmware blobs or vendor assets. Track those separately in `THIRD_PARTY_NOTICES.md`.
