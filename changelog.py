#!/usr/bin/env python

from argparse import ArgumentParser
import git
import re


maintainer = "lukkash@email.cz"


def format_entry(entry, author):
    entry = re.sub(" +> +", "\n  \n  ", entry)

    if author.email != maintainer:
        if "\n" in entry:
            return entry.replace("\n", " [{}]\n".format(author.name), 1)
        else:
            return entry + " [{}]".format(author.name)

    return entry


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
                features.append(format_entry(commit.trailers["Feature"], commit.author))

            if "Fix" in commit.trailers:
                fixes.append(format_entry(commit.trailers["Fix"], commit.author))

    def print_list(lst):
        for entry in reversed(lst):
            print("- {}\n".format(entry))

    if features:
        print("### Features")
        print_list(features)

    if fixes:
        print("### Fixes")
        print_list(fixes)


if __name__ == '__main__':
    main()
