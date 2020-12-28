/* Size of mailbox used to transfer between host and target driver
 * DO NOT CHANGE */
__mailbox_size = 24;

GROUP(libcr_c.a libcr_eabihelpers.a)

/* Ensure items from FlashDriverLib are pulled in */
EXTERN (FlashDevice MemoryDevice ServiceMessages)

SEARCH_DIR(..)

SECTIONS
{
	ENTRY(ServiceMessages)						
	
	.cr_data :
	{
		CREATE_OBJECT_SYMBOLS
		_b_cr_data = .;
		__load_base = _b_cr_data;
		KEEP(* (.MemoryDevice))
		_e_cr_data = .;
	} > SRAM
		
	.text :
	{
		CREATE_OBJECT_SYMBOLS
		_b_text = .;
		KEEP(*ServiceMessages.o(.text*))
		*(.text*)
		*(.rodata*)
		_e_text = .;		
	} > SRAM

	/*
		Flash Device
	*/
	.data :
	{
		CREATE_OBJECT_SYMBOLS
		_b_data = .;
		*(.data*)
		_e_data = .;
		_b_bss = .;		
		*(.bss*)
		*( COMMON )
		_e_bss = .;
		_b_rodata = _e_data;

	    /* Ensure FlashDevice structure is last thing in image */ 						
	    _b_flashdev = .;
	    KEEP(*(.FlashDevice))
	    _e_flashdev = .;
	    
	    _e_image = .;
		. = ALIGN(8);
	    _e_image_padded = .;
	} > SRAM
	
  	__image_size = _e_image - _b_cr_data; 

	.stackcache (NOLOAD):
	{
		_b_stack = . ;
		. = . + __stack_size;
		_e_stack = .;
		__initial_sp = . ;
		_b_cache = __initial_sp;
		__cache = _b_cache;
		. = . +  __cache_size; 
		_e_cache = .;
		_b_mailbox = . ;
		. = . + __mailbox_size;
		_e_mailbox = . ;	
	} > SRAM2


}

