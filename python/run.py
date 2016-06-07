#!/usr/bin/python3

import os, sys, argparse, subprocess
from TestRunner import TestRunner

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    testing= parser.add_argument_group('testing', '')
    testing.add_argument("--loss", type=int, nargs='+', help="run loss tests for given values [%%]")
    testing.add_argument("--delay", type=int, nargs='+', help="run delay tests for given values [ms]")
    testing.add_argument("--limit", type=int, nargs='+', help="run limit tests for given values [kbit]")
    testing.add_argument("--duplication", type=int, nargs='+', help="run duplication tests for given values [%%]")
    testing.add_argument("--corruption", type=int, nargs='+', help="run corruption tests for given values [%%]")
    testing.add_argument("--reorder", type=int, nargs='+', help="run reorder tests for given values [%%]")
    testing.add_argument("--test", choices=[ 'ros1', 'ros2', 'opensplice' ], nargs='+', help="Transport layer to be tested")
    tools = parser.add_argument_group('tools', '')
    tools.add_argument("--build-all", action='store_true', help ="build all images")
    tools.add_argument("--build", choices=TestRunner.images, nargs='+', help="delete an existing image and build a new one")
    tools.add_argument("--replot", action='store_true', help ="plot again current results")
    tools.add_argument("--clean", action='store_true', help ="stop and remove all containers")
    tools.add_argument("--qtcreator", choices=[ 'ros1', 'ros2', 'opensplice' ], help="Run QT Creator in the development environment")
    if len(sys.argv) == 1:
        parser.print_help()
    else:
        args = parser.parse_args()
        runner = TestRunner()
        if args.build:
            runner.clean()
            for image in args.build:
                subprocess.call("./scripts/build_container.sh {}".format(image), shell = True)
        elif args.build_all:
            runner.clean()
            for name in reversed(TestRunner.images):
                subprocess.call("./scripts/remove_container.sh {}".format(name), shell = True)
            for name in TestRunner.images:
                subprocess.call("./scripts/build_container.sh {}".format(name), shell = True)
        elif args.replot:
            subprocess.call("sh ./graphs/replot.sh", shell = True)
        elif args.qtcreator:
            subprocess.call("./scripts/qtcreator.sh {}".format(args.qtcreator), shell = True)
        elif args.clean:
            runner.clean()
        else:
            if not os.geteuid() == 0:
                sys.exit("Only root can run tests")
            for comm in args.test:
                if args.limit:
                    runner.limit(comm, args.limit)
                if args.duplication:
                    runner.duplication(comm, args.duplication)
                if args.corruption:
                    runner.corruption(comm, args.corruption)
                if args.reorder:
                    runner.reorder(comm, args.reorder)
                if args.loss:
                    runner.loss(comm, args.loss)
                if args.delay:
                    runner.delay(comm, args.delay)
            runner.clean()
