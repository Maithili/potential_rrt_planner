/** \file util_shparse.c
 * \brief Implementation of cd_util_shparse, a simple POSIX shell argument
 *        parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012-2013 Christopher Dellin */

/* This module (cd_util_shparse) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 */

/* An implementation of simple POSIX shell argument parsing.
 * Written by Chris Dellin, 2012-07-03
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util_shparse.h"

int cd_util_shparse(char * in, int * argcp, char *** argvp)
{
   int inarg;
   char quot;
   int i;
   int skipped;
   int argc;
   int argi;
   char ** argv;
   
   /* first, figure out how many arguments there are */
   argc = 0;
   inarg = 0;
   quot = 0;
   skipped = 0;
   for (i=0; in[i]; i++)
   {
      /* swallow spaces inter-arg */
      if (!inarg)
      {
         if (isspace(in[i]))
         {
            in[i] = 0;
            continue;
         }
         inarg = 1;
         argc++;
         skipped = 0;
      }
      /* are we ending an argument? */
      if (inarg && !quot && isspace(in[i]))
      {
         for (; skipped >= 0; skipped--)
            in[i-skipped] = 0;
         inarg = 0;
         continue;
      }
      /* are we entering / leaving quotes? */
      if (inarg && !quot && (in[i] == '"' || in[i] == '\''))
      {
         quot = in[i];
         skipped++;
         continue;
      }
      if (inarg && quot && in[i] == quot)
      {
         quot = 0;
         skipped++;
         continue;
      }
      /* is this a backslash escape? */
      if (inarg && (!quot || quot == '"') && in[i] == '\\' && in[i+1])
      {
         if (in[i+1] == '\n')
         {
            i++;
            skipped += 2;
            continue;
         }
         if (!quot || in[i+1] == '"' || in[i+1] == '\\')
         {
            i++;
            skipped++;
         }
      }
      /* copy the character! */
      in[i-skipped] = in[i];
   }
   /* copy in necessary null at the end */
   for (; skipped >= 0; skipped--)
      in[i-skipped] = 0;
   
   argv = (char **) malloc(argc * sizeof(char *));
   if (!argv) return -1;
   
   inarg = 0;
   for (argi=0, i=0; argi<argc; i++)
   {
      if (!inarg && in[i])
      {
         inarg = 1;
         argv[argi] = in+i;
         argi++;
      }
      if (inarg && !in[i])
         inarg = 0;
   }
   
   *argcp = argc;
   *argvp = argv;
   return 0;
}
