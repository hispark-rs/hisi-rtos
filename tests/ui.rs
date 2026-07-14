#[test]
fn runtime_mode_capabilities_are_checked_at_compile_time() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/cooperative_policy.rs");
    tests.pass("tests/ui/ported_policy.rs");
}
