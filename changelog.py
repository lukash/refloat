#!/usr/bin/env python

from argparse import ArgumentParser
import git
import re


def main():
    parser = ArgumentParser(prog='changelog')
    args = parser.parse_args()

    repo = git.Repo(".", search_parent_directories=True)

    tagmap = {}
    for t in repo.tags:
        tagmap.setdefault(repo.commit(t), []).append(t)

    def ref_tag(commit):
        for tag in tagmap.get(commit, tuple()):
            if re.match("^v[0-9]+\.[0-9]+\.[0-9]+.*", tag.name):
                return tag

        return None

    first_version = ref_tag(repo.head.commit)

    features = []
    fixes = []
    for commit in repo.iter_commits():
        if commit != repo.head.commit:
            tag = ref_tag(commit)
            if tag is not None:
                break

        if commit.trailers:
            if "Feature" in commit.trailers:
                features.append(commit.trailers["Feature"])

            if "Fix" in commit.trailers:
                fixes.append(commit.trailers["Fix"])

    def print_list(lst):
        for item in reversed(lst):
            with_linebreaks = re.sub(" *> *", "\n  \n  ", item)
            print("- {}\n".format(with_linebreaks))

    print("### Features")
    print_list(features)
    print("### Fixes")
    print_list(fixes)



if __name__ == '__main__':
    main()
