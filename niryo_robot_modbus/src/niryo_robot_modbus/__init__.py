def safe_get(container, item, default_value):
    """
    Safely retrieves the value at the specified item from the given container.

    This function attempts to access the element at the specified ``item``
    within the provided ``container``. If the item is not found,
    it returns the specified ``default_value`` instead.

    :param container: The container to retrieve the value from (list or dict).
    :type container: any object implementing the __getitem__ method
    :param item: The item to access.
    :type item: Any
    :param default_value: The value to return if the item is not found.
    :type default_value: Any
    :return: The value at the specified item within the container, or ``default_value`` if the item is not found.
    :rtype: Any
    """
    try:
        return container[item]
    except (IndexError, KeyError):
        return default_value
