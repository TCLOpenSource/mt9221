#!/bin/perl
##
## /* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
## /******************************************************************************
##  *
##  * This file is provided under a dual license.  When you use or
##  * distribute this software, you may choose to be licensed under
##  * version 2 of the GNU General Public License ("GPLv2 License")
##  * or BSD License.
##  *
##  * GPLv2 License
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  *
##  * This program is free software; you can redistribute it and/or modify
##  * it under the terms of version 2 of the GNU General Public License as
##  * published by the Free Software Foundation.
##  *
##  * This program is distributed in the hope that it will be useful, but
##  * WITHOUT ANY WARRANTY; without even the implied warranty of
##  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##  * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
##  *
##  * BSD LICENSE
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  * All rights reserved.
##  *
##  * Redistribution and use in source and binary forms, with or without
##  * modification, are permitted provided that the following conditions
##  * are met:
##  *
##  *  * Redistributions of source code must retain the above copyright
##  *    notice, this list of conditions and the following disclaimer.
##  *  * Redistributions in binary form must reproduce the above copyright
##  *    notice, this list of conditions and the following disclaimer in
##  *    the documentation and/or other materials provided with the
##  *    distribution.
##  *  * Neither the name of the copyright holder nor the names of its
##  *    contributors may be used to endorse or promote products derived
##  *    from this software without specific prior written permission.
##  *
##  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
##  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
##  * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
##  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
##  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
##  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
##  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
##  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
##  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##  *
##  *****************************************************************************/
##
##
##
## /* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
## /******************************************************************************
##  *
##  * This file is provided under a dual license.  When you use or
##  * distribute this software, you may choose to be licensed under
##  * version 2 of the GNU General Public License ("GPLv2 License")
##  * or BSD License.
##  *
##  * GPLv2 License
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  *
##  * This program is free software; you can redistribute it and/or modify
##  * it under the terms of version 2 of the GNU General Public License as
##  * published by the Free Software Foundation.
##  *
##  * This program is distributed in the hope that it will be useful, but
##  * WITHOUT ANY WARRANTY; without even the implied warranty of
##  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##  * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
##  *
##  * BSD LICENSE
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  * All rights reserved.
##  *
##  * Redistribution and use in source and binary forms, with or without
##  * modification, are permitted provided that the following conditions
##  * are met:
##  *
##  *  * Redistributions of source code must retain the above copyright
##  *    notice, this list of conditions and the following disclaimer.
##  *  * Redistributions in binary form must reproduce the above copyright
##  *    notice, this list of conditions and the following disclaimer in
##  *    the documentation and/or other materials provided with the
##  *    distribution.
##  *  * Neither the name of the copyright holder nor the names of its
##  *    contributors may be used to endorse or promote products derived
##  *    from this software without specific prior written permission.
##  *
##  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
##  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
##  * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
##  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
##  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
##  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
##  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
##  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
##  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##  *
##  *****************************************************************************/
##
##

	my $folder = $ARGV[0];
    my $full_folder = "";
    if($folder =~ /^\//){
        $full_folder = $folder;
    }
    else{
        $full_folder = `pwd`."/".$folder;
    }
    $full_folder =~ s/\n//;
	my @git_list=`find $full_folder -type d -name '.git' | grep -v repo`;
	my $last_id = "";
	my $last_time = "";
	my $short_dir = "";
	if( -f "$full_folder/revision.sh" ){
		system("rm $full_folder/revision.sh");
	}
	foreach my $gl (0..$#git_list)
	{
		#print $git_list[$gl]."\n";
		my $project_dir = substr $git_list[$gl], 0, -5;
		#print "real dir: $project_dir\n";
		if($project_dir =~ /$folder\/(.+?)$/)
		{
			$short_dir = $1;
		}
		chdir($project_dir) or die "$!";
		my $commitID = `git log -n 1 --format="%H"`;
		$commitID =~ s/\n//g;
		my $cmd = "echo \"cd $short_dir\ngit checkout -f $commitID\ncd -\" >> $full_folder/revision.sh";
		#print "command: $cmd\n";
		system($cmd);
		my $commitTime = `git log -n 1 --format="%ct"`;
		if( $last_time eq "" || $last_time < $commitTime)
		{
			$last_time = $commitTime;
			$last_id = $commitID;
		}
	}
	print substr($last_id,0,7)."\n";
