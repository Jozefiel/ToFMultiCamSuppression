#!/bin/bash

__BASE_PATH=`dirname $(realpath $0)`

num_array=(0.4 0.6 0.8)
buffer=(3 4 5 6 7 8)
max_sample=98

#cp /home/jozefiel/Documents/QT/ToFMultiCamSuppression_build/Interference ${__BASE_PATH} 
echo "Please copy binary (from build) with name 'Interference' to ${__BASE_PATH} and hit Enter!!!"
read var1

mkdir -p ${__BASE_PATH}/results


function collectData() {

cam_num=${1}
pushd ${__BASE_PATH}/${cam_num}
    for i in {0..98}
    do
	    for num in "${buffer[@]}"
	    do
        mkdir -p ${num}/${i}/data
        counter=0
            while [ ${counter} -ne ${num} ]
            do
                _filecnt=$((i + counter))
                counter=$((counter+1))
	            cp "${cam_num}_depth${_filecnt}.txt" ${num}/${i}/ > /dev/null
            done
        done  
    done
popd
}

function filtration() {

cam_num=${1}
pushd ${__BASE_PATH}/${cam_num}
    ref_rgb="ref.png"

    for tmp in "${buffer[@]}" 
    do
	    for num in "${num_array[@]}"
	    do
		    mkdir -pv ${tmp}/results/${num}
		    for i in {1..96} 
		    do
        		${__BASE_PATH}/Interference ${tmp}/${i}/ ../../${ref_rgb} ${num} >> time_${cam_num}.txt
        		cp ${tmp}/${i}/data/median.ply ${tmp}/results/${num}/${i}.ply
        		cp ${tmp}/${i}/data/median.txt ${tmp}/results/${num}/${i}.txt
		    done
	    done
    done
}

function computeHausdorff() {

cam_num=${1}
ref_model=${2}
if [[ ! -z "${ref_model}" ]]; then
	ref_cloud_ply="${ref_model}"
	ref_cloud_pcd="${ref_model}.pcd"
else
	ref_cloud_ply="${__BASE_PATH}/../ref/${cam_num}/data/median.ply"
   	ref_cloud_pcd="${__BASE_PATH}/../ref/${cam_num}/data/median.pcd"
fi
pushd ${__BASE_PATH}/${cam_num}


    pcl_ply2pcd ${ref_cloud_ply} ${ref_cloud_pcd}
    
    for tmp in "${buffer[@]}" 
    do
	    for num in "${num_array[@]}"
	    do
		    mkdir -p ${tmp}/results/${num}/pcd
            echo "" > ${tmp}/results/${num}/pcd/result.txt 
		    for i in {1..90} 
		    do
			    pcl_ply2pcd ${tmp}/results/${num}/${i}.ply ${tmp}/results/${num}/pcd/${i}.pcd > /dev/null
			    pcl_compute_hausdorff ${tmp}/results/${num}/pcd/${i}.pcd ${ref_cloud_pcd} 2>&1 | grep -i "A->B" >> ${tmp}/results/${num}/pcd/result_${cam_num}_${tmp}_${num}.txt 
		    done
	    done
    done
popd
}

function resultsCollecting() {

cam_num=${1}
pushd ${__BASE_PATH}/${cam_num}
    mkdir -p results
    find . -type f -iname "result_*.txt" -exec cp {} results \; 

    find results -type f -iname "result_*.txt" -exec sed -i 's/[^0-9.:]*//g' {} \;
    find ./results -type f -iname "result_*.txt" -exec sed -i 's/\:\:/\:/g' {} \;

    FILES="results/result_*"
    for f in ${FILES}
    do
      cat  $f | cut -d ':' -f4 > results/.tmp.txt | mv results/.tmp.txt $f
      echo $f >> $f
    done

    paste ./results/result_* > ${__BASE_PATH}/results/haus_result_${cam_num}.txt
popd
}

function main () {

    camera=(0 1 2)
    for f in "${camera[@]}"
    do
        if [[ -d ${f} ]]; then

                __PATH=$(pwd)
                collectData ${f}
                filtration ${f}
                computeHausdorff ${f}
                resultsCollecting ${f}
            popd > /dev/null
            
        fi
    done
}

"$@"





