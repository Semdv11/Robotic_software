#!/bin/bash 
# шебанг(#!), после которого идет имя файла программы-интерпретатора

# Функция поиска файлов в определенной директории
file_info () {

    # Выбираем разделитилем символ перевода строки
    IFS=$'\n'
    
    # В цикле проходимся по каждому имени из вывода команды ls
    for element in $(ls $1) 
    do
	# При появлении директории рекурсивно запускаем функцию поиска файлов в ней
	if [ -d "$1/$element" ]; then
	echo "$1/$element"
	file_info "$1/$element"
     	#В ином случае выводим информацию о файле
	else
	    # Для медиа-файлов сохраняется длительность
	    if [ "${element##*"${element%.*}"}" == ".txt" ]
	    then
            time=""
            else
	    time="$(ffmpeg -i $1"/"$element 2>&1 | grep Duration | cut -d ' ' -f 4 | tr -d ,)"
            fi 
 
	    #Чтобы избежать вывода N/A для изображений в поле "duration" за неспособностью вывести длительность картинки, вводим условие
	    if [ "$time" == "N/A" ]
	    then
		time=""
	    fi 		

            # Компонуем всю инофрмацию о файле в одну строку, переводя размер файла из Кб в Мб
	    size_celoe=$(( $(stat -c %s $1"/""$element")/1024 ))
	    size_ostatok=$(( $(stat -c %s $1"/""$element")%1024 ))
	    full=${element%.*}","${element##*"${element%.*}"}",$size_celoe"."$size_ostatok,"$(stat -c %z $1"/""$element")","$time
	   
            echo $full >> ~/Documents/PO/lab1/folder/lab1.csv  # Отправляем сформированную строку в файл
	fi
    done
}

# Создаем шапку таблицы
echo "name, format, size(Mb), date, duration (HH:MM:SS)" > lab1.csv
file_info "$(pwd)"
