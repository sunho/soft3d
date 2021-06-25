cmake_minimum_required(VERSION 3.10)

function(create_resources dir output)
    # Create empty output file
    file(WRITE ${output} "")
    # Collect input files
    file(GLOB bins ${dir}/*)
    # Iterate through input files
    foreach(bin ${bins})
        # Get short filename
        message(${bin})
        string(REGEX MATCH "([^/]+)$" filename ${bin})
        # Replace filename spaces & extension separator for C compatibility
        string(REGEX REPLACE "\\.| |-" "" filename ${filename})
        # Read hex data from file
        file(READ ${bin} filedata HEX)
        # Convert hex data for C compatibility
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})
        # Append data to output file
        file(APPEND ${output} "const unsigned char ${filename}[] = {${filedata}};\nconst unsigned ${filename}Size = sizeof(${filename});\n")
    endforeach()
endfunction()