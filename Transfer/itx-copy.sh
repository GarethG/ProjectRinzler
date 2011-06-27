sudo rsync -azvv -e ssh ~/ProjectRinzler/ uwesub@192.168.2.30:/home/uwesub/ProjectRinzler/ --exclude-from 'itx-exlude.txt'
