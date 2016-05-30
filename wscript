#! /usr/bin/env python

def options(ctx):
    ctx.add_option('--foo', action='store', default=False, help='Silly test')

def configure(ctx):
    print('→ the value of foo is %r' % ctx.options.foo)
