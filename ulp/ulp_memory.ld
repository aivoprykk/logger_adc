ld
MEMORY {
  rtc_slow_mem (rwx) : ORIGIN = 0x50000000, LENGTH = 0x2000
}

SECTIONS {
  .text : {
    _text_start = .;
    *(.text)
    _text_end = .;
  } > rtc_slow_mem
  .data : {
    _data_start = .;
    *(.data)
    _data_end = .;
  } > rtc_slow_mem
  .bss : {
    _bss_start = .;
    *(.bss)
    _bss_end = .;
  } > rtc_slow_mem
  .rtc_variables : {
    /* Ensure these symbols are placed in RTC memory */
    _low_thr = .;
    *(.low_thr)
    _high_thr = .;
    *(.high_thr)
    _sample_counter = .;
    *(.sample_counter)
    _last_result = .;
    *(.last_result)
  } > rtc_slow_mem
}