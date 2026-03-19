from berkeley_humanoid_lite_assets.robots import (
    BIPED_VARIANT,
    FULL_BODY_VARIANT,
    get_variant,
    get_variant_for_joint_count,
)


def test_get_variant_returns_standard_variants():
    assert get_variant("biped") == BIPED_VARIANT
    assert get_variant("full_body") == FULL_BODY_VARIANT


def test_get_variant_for_joint_count_resolves_variant():
    assert get_variant_for_joint_count(BIPED_VARIANT.joint_count) == BIPED_VARIANT
    assert get_variant_for_joint_count(FULL_BODY_VARIANT.joint_count) == FULL_BODY_VARIANT
