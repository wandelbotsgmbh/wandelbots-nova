import datetime as dt


def now_utc() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def datetime_to_rfc3339(dt: dt.datetime) -> str:
    return dt.isoformat(timespec="milliseconds")


def now_rfc3339() -> str:
    return datetime_to_rfc3339(now_utc())
