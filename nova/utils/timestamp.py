import datetime as dt


def now_rfc3339() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
