from wandelscript import Store


def test_store():
    store = Store(init_vars={"a": 1, "b": 2, "r": {"x": 1, "y": 2}})
    assert store.data_dict == {"a": 1, "b": 2, "r": {"x": 1, "y": 2}}
