import csv
import sys

from cpppo.server import enip
from cpppo.server.enip import client, Object, config_files
from cpppo.server.enip.main import tags, main as enip_main

"""
Ethernet/IP Server Simulator
Based on cpppo server example:
https://github.com/pjkundert/cpppo/blob/master/server/enip/simulator_example.py
"""

def main( argv=None, idle_service=None, **kwds ):
    if argv is None:
        argv			= sys.argv[1:]

    # Remember any tags we find w/ optional values CSV.  We won't know the type 'til after the
    # Attribute is created by enip.main, so just store the raw CSV for now.  Use the Object class'
    # config_loader, and only look for [Simulator] entries (don't use DEFAULT).  Iterate thru all
    # the entries, adding an entry to kwds for each.
    values			= {} # All collected Tags w/ a CSV of initial values

    # Load config_files early (with case sensitivity); enip.main will re-do it, case-insensitively
    optionxform			= Object.config_loader.optionxform
    Object.config_loader.optionxform = str # Case-sensitive
    Object.config_loader.read( config_files )

    if 'Simulator' in Object.config_loader:
        for nam,typ in Object.config_loader['Simulator'].items():
            val			= None
            if '=' in typ:
                # Optional value(s)
                typ,val		= typ.split( '=', 1 )
                typ,val		= typ.strip(),val.strip()
            argv		       += [ '='.join( (nam,typ) ) ] # eg. Tag@123/4/5=REAL[100]
            if val:
                # A non-empty initial value was provided; strip off any optional CIP address and
                # save the initial values provided.
                if '@' in nam:
                    nam		= nam[:nam.index( '@' )]
                values[nam]	= val

    Object.config_loader.optionxform = optionxform
    Object.config_loader.clear()

    def idle_init():
        """First time thru, set up any initia values; subsequently, perform original idle_service."""
        if idle_init.complete:
            if idle_service:
                print(values)
                idle_service()
            return
        idle_init.complete	= True
        for nam in values:
            # Got initial value(s) for this one.
            val_list		= []
            try:
                val_list,	= csv.reader(
                    [ values[nam] ], quotechar='"', delimiter=',', quoting=csv.QUOTE_ALL, skipinitialspace=True )
                ent		= dict.__getitem__( tags, nam ) # may be 'Tag.SubTag'; avoid dotdict '.' resolution
                typ		= ent.attribute.parser.__class__.__name__ # eg. 'REAL'
                _,_,cast	= client.CIP_TYPES[typ]
                ent.attribute[0:len( val_list )] \
                                    = [ cast( v ) for v in val_list ]
            except Exception:
                print( "Failed to set %s[0:%d] = %r" % ( nam, len( val_list ), val_list ))
                raise
    idle_init.complete		= False

    # Establish Identity, TCPIP, etc. objects, and any custom [Simulator] tags from the config file(s).
    return enip_main( argv=argv, idle_service=idle_init, **kwds )


if __name__ == "__main__":
    # For demonstration, use .../simulator_example.cfg
    enip.config_files          += [ __file__.replace( '.py', '.cfg' ) ]
    sys.exit( main() )
