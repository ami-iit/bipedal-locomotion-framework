import pytest
pytestmark = pytest.mark.parameters_handler

import bipedal_locomotion_framework.bindings.parameters_handler as blf
import numpy as np


def test_bool():

    handler = blf.StdParametersHandler()

    handler.set_parameter_bool(name="my_bool", value=True)

    assert handler.get_parameter_bool(name="my_bool") is True

    with pytest.raises(ValueError):
        handler.get_parameter_int(name="my_bool")

    with pytest.raises(ValueError):
        handler.get_parameter_float(name="my_bool")

    with pytest.raises(ValueError):
        handler.get_parameter_string(name="my_bool")


def test_int():

    handler = blf.StdParametersHandler()

    handler.set_parameter_int(name="my_int", value=42)

    assert handler.get_parameter_int(name="my_int") == 42

    with pytest.raises(ValueError):
        handler.get_parameter_bool(name="my_int")

    with pytest.raises(ValueError):
        handler.get_parameter_float(name="my_int")

    with pytest.raises(ValueError):
        handler.get_parameter_string(name="my_int")


def test_float():

    handler = blf.StdParametersHandler()

    handler.set_parameter_float(name="my_float", value=3.1415)

    assert handler.get_parameter_float(name="my_float") == pytest.approx(3.1415)

    with pytest.raises(ValueError):
        handler.get_parameter_bool(name="my_float")

    with pytest.raises(ValueError):
        handler.get_parameter_int(name="my_float")

    with pytest.raises(ValueError):
        handler.get_parameter_string(name="my_float")


def test_string():

    handler = blf.StdParametersHandler()

    handler.set_parameter_string(name="my_string", value="foo")

    assert handler.get_parameter_string(name="my_string") == "foo"

    with pytest.raises(ValueError):
        handler.get_parameter_bool(name="my_string")

    with pytest.raises(ValueError):
        handler.get_parameter_int(name="my_string")

    with pytest.raises(ValueError):
        handler.get_parameter_float(name="my_string")


def test_vector_bool():

    handler = blf.StdParametersHandler()

    handler.set_parameter_vector_bool(name="my_vector_bool",value= [True, False, True])

    assert handler.get_parameter_vector_bool(name="my_vector_bool") == [True, False, True]

    with pytest.raises(ValueError):
        handler.get_parameter_vector_int(name="my_vector_bool")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_float(name="my_vector_bool")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_string(name="my_vector_bool")


def test_vector_int():

    handler = blf.StdParametersHandler()

    handler.set_parameter_vector_int(name="my_vector_int", value=[-1, 2, 10])

    assert handler.get_parameter_vector_int(name="my_vector_int") == [-1, 2, 10]

    with pytest.raises(ValueError):
        handler.get_parameter_vector_bool(name="my_vector_int")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_float(name="my_vector_int")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_string(name="my_vector_int")


def test_vector_float():

    handler = blf.StdParametersHandler()

    handler.set_parameter_vector_float(name="my_vector_float",
                                       value=[-3.14, 2.7182, 42.0])

    assert handler.get_parameter_vector_float(name="my_vector_float") == \
           pytest.approx([-3.14, 2.7182, 42.0])

    with pytest.raises(ValueError):
        handler.get_parameter_vector_bool(name="my_vector_float")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_int(name="my_vector_float")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_string(name="my_vector_float")


def test_vector_string():

    handler = blf.StdParametersHandler()

    handler.set_parameter_vector_string(name="my_vector_string",
                                        value=["foo", "bar", "bipedal", "locomotion"])

    assert handler.get_parameter_vector_string(name="my_vector_string") == \
           ["foo", "bar", "bipedal", "locomotion"]

    with pytest.raises(ValueError):
        handler.get_parameter_vector_bool(name="my_vector_string")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_int(name="my_vector_string")

    with pytest.raises(ValueError):
        handler.get_parameter_vector_float(name="my_vector_string")


def test_vector_mixed():

    handler = blf.StdParametersHandler()

    # 1. Mixed vector: store as more general type float
    handler.set_parameter_vector_float(name="to_float", value=[42.0, 1, -3.14, False])

    assert handler.get_parameter_vector_float(name="to_float") == \
           pytest.approx([42.0, 1.0, -3.14, 0.0])

    # 2. Mixed vector: store as more general type int
    handler.set_parameter_vector_float(name="to_int", value=[42, 1, -3, False])

    assert handler.get_parameter_vector_float(name="to_int") == \
           pytest.approx([42, 1, -3, 0])

    # 3. Mixed vector: store as less general type int
    with pytest.raises(TypeError):
        handler.set_parameter_vector_int(name="to_int_fail",
                                         value=[42.0, 1, -3.14, False])


def test_clear():

    handler = blf.StdParametersHandler()

    handler.set_parameter_bool(name="my_bool1", value=False)
    handler.set_parameter_bool(name="my_bool2", value=True)
    handler.set_parameter_float(name="my_float", value=-42.42)
    handler.set_parameter_vector_string(name="my_vector_string", value=["bar", "foo"])

    handler.clear()

    with pytest.raises(ValueError):
        _ = handler.get_parameter_bool(name="my_bool1")

    with pytest.raises(ValueError):
        _ = handler.get_parameter_bool(name="my_bool2")

    with pytest.raises(ValueError):
        _ = handler.get_parameter_float(name="my_float")

    with pytest.raises(ValueError):
        _ = handler.get_parameter_vector_string(name="my_float")
