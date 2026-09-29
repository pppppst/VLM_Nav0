from pathlib import Path


SPARK_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "go2_ws/src/spark-fast-lio/spark_fast_lio/src/spark_fast_lio.cpp"
)


def test_spark_has_one_odometry_publish_path():
    calls = [
        line
        for line in SPARK_SOURCE.read_text(encoding="utf-8").splitlines()
        if "publishOdometry(" in line and "void SPARKFastLIO2::publishOdometry" not in line
    ]

    assert len(calls) == 1, calls
