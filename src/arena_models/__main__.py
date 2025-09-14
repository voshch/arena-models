import sys


def main():
    verb = sys.argv.pop(1) if len(sys.argv) > 1 else None

    if verb == "down":
        from .down_db import main as down_db_main
        down_db_main()
    elif verb == "build":
        from .build_db import main as build_db_main
        build_db_main()
    elif verb == 'get':
        from .get_db import main as get_db_main
        get_db_main()
    elif verb == 'merge':
        from .merge_db import main as merge_db_main
        merge_db_main()
    elif verb == 'query':
        from .query_db import main as query_db_main
        query_db_main()
    else:
        raise ValueError(f"Unknown verb: {verb}. Available verbs: down, build, get, merge, query.")


if __name__ == "__main__":
    main()
