################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
source/%.obj: ../source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/dsp/FixedPoint/c28/examples/common" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/ccs1200/ccs/ccs_base/c2000/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/dsp/FixedPoint/c28/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/driverlib/f28003x/driverlib" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/device_support/f2837xd/common/include" --advice:performance=all --define=_FLASH --define=CPU1 --abi=eabi --preproc_with_compile --preproc_dependency="source/$(basename $(<F)).d_raw" --obj_directory="source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


