# AGENTS.md

## UI Declarative Style Rules (creeper-qt)

1. For local UI builder/helper scopes (e.g., `make_*` helpers), start the scope with:
   - `using namespace creeper;`

2. Use only the target component's property namespace when constructing declarative props:
   - e.g. `text_field::pro::*` for `OutlinedTextField`, `slider::pro::*` for `Slider`, `card::pro::*` for `FilledCard`.
   - Do not mix unrelated property namespaces in the same component property chain.
   - Exception: dedicated namespace groups intentionally designed as standalone groups.

3. Do **not** use internal/common wrapper hacks for missing props:
   - Forbidden: `common::pro::*` template workarounds and `*::internal::*` property wrapper shortcuts.
   - If a declarative property is unavailable, use `Apply` and set value directly on the widget instance.
