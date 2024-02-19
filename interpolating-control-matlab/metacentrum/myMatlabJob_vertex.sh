#!/bin/bash
#PBS -N UAV-vertex-ctrl
#PBS -l walltime=6:00:00
#PBS -l select=1:ncpus=6:mem=16gb:scratch_local=20gb
#PBS -l matlab=1
#PBS -l matlab_Distrib_Computing_Toolbox=1
#PBS -q	iti
#PBS -W group_list=iti
#PBS -j oe
#PBS -m e

# nastaveni domovskeho adresare, v promenne $LOGNAME je ulozeno vase prihlasovaci jmeno
DATADIR="/storage/plzen1/home/$LOGNAME/matlab"

# nacteni aplikacniho modulu, ktery zpristupni aplikaci Gaussian verze 3
module add matlab

# nastaveni automatickeho vymazani adresare SCRATCH pro pripad chyby pri behu ulohy
trap 'clean_scratch' TERM EXIT

# checks if scratch directory is created
if [ ! -d "$SCRATCHDIR" ] ; then echo "Scratch directory is not created!" 1>&2; exit 1; fi

# vstup do adresare SCRATCH, nebo v pripade neuspechu ukonceni s chybovou hodnotou rovnou 1
cd $SCRATCHDIR || exit 1

# priprava vstupnich dat (kopirovani dat na vypocetni uzel)
cp -r $DATADIR $SCRATCHDIR

cd matlab
# spusteni aplikace - samotny vypocet
matlab -nodesktop -r "run run_mpt_script_vertex_ctrl;"

# kopirovani vystupnich dat z vypocetnicho uzlu do domovskeho adresare,
# pokud by pri kopirovani doslo k chybe, nebude adresar SCRATCH vymazan pro moznost rucniho vyzvednuti dat
cd $SCRATCHDIR/matlab/interpolating_control
cp -a output/. $DATADIR/vertex_ctrl/ || export CLEAN_SCRATCH=false
