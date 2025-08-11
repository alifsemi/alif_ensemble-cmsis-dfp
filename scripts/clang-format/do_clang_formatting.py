import subprocess
import sys
import re
from   pathlib import Path

#find . -type f \( -name "*.c" -o -name "*.h" \) | wc -l

# Match multiline braced initializers (elements with commas, ending in };)
pattern = re.compile(
    r"""
    (?P<equals>\s*=\s*)\{              # group 'equals': match = with optional spaces
    \s*                                # optional space/newline after {
    (?P<first>[a-zA-Z_][a-zA-Z0-9_]*),\s*\n  # group 'first': first element
    (?P<middle>(?:\s*[a-zA-Z_][a-zA-Z0-9_]*,\s*\n)+)  # group 'middle': comma lines
    \s*(?P<last>[a-zA-Z_][a-zA-Z0-9_]*)\s*  # group 'last': final item
    \};                                 # closing
    """, re.VERBOSE
)

pattern1 = re.compile(r'\breturn\s*\(\s*([a-zA-Z0-9_]+)\s*\)\s*;')
pattern2 = re.compile(r'\+\s*([0-9])')

# Match static declarations with = 0, 0U, false, or NULL
pattern3 = re.compile(
    r'''(?x)                                # enable verbose mode
    (^ \s* static \s+ [^=;\n]+?)            # group 1: start of static declaration up to '='
    \s* = \s*
    (0U|0|false|NULL)                       # group 2: known redundant values
    \s* ;                                   # semicolon
    '''
)

# Pattern: return (func(args));
pattern4 = re.compile(r'\breturn\s*\(\s*([a-zA-Z_][a-zA-Z0-9_]*\s*\([^;]*?\))\s*\)\s*;')

def reformat_initializer_block(match):
    equals = match.group("equals")
    first  = match.group("first")
    middle = match.group("middle")
    last   = match.group("last")

    # Clean and indent all lines
    lines = [line.strip() for line in middle.strip().splitlines()]
    lines = ['    ' + line for line in lines]  # 4 spaces

    # Add the last line
    lines.append(f'    {last}')

    # Join the formatted block
    return f"{equals}{{\n" + f"    {first.strip()},\n" + '\n'.join(lines) + "\n};"


# Custom function to process each file
def process_file(file_path: Path):
    print(f"Processing file: {file_path}")

    #Run clang-format in-place
    result = subprocess.run(
       ["clang-format", "-style=file:.clang-format", "-i", str(file_path)],
       capture_output=True,
       text=True
    )

    if result.returncode != 0:
        print(f"Error formatting {file_path}:\n{result.stderr}")
    else:
        print(f"Formatted  file: {file_path}")

    with file_path.open("r", encoding="utf-8", errors="ignore") as f:
        line_count = sum(1 for _ in f)
        f.seek(0)
        content = f.read()

        new_content = pattern.sub(reformat_initializer_block, content)

        if new_content != content:
            print(f"Array Adjustment: {file_path}")
            with file_path.open("w", encoding="utf-8") as f:
                f.write(new_content)

        new_content1 = pattern1.sub(r'return \1;', new_content)

        if new_content1 != new_content:
            print(f"Return Statement: {file_path}")
            with file_path.open("w", encoding="utf-8") as f:
                f.write(new_content1)

        #new_content2 = pattern2.sub(r'\1', new_content1)

        # if new_content2 != new_content1:
            # print(f"Unary Operator: {file_path}")
            # with file_path.open("w", encoding="utf-8") as f:
                # f.write(new_content2)
        new_content2 = new_content1

        new_content3 = pattern3.sub(r'\1;', new_content2)

        if new_content3 != new_content2:
            print(f"Static Adjustment: {file_path}")
            with file_path.open("w", encoding="utf-8") as f:
                f.write(new_content3)

        new_content4 = pattern4.sub(r'return \1;', new_content3)

        if new_content4 != new_content3:
            print(f"Return_one: {file_path}")
            with file_path.open("w", encoding="utf-8") as f:
                f.write(new_content4)

    #print(f"{file_path}: {line_count} lines")

    return line_count

# Recursively scan for .c and .h files
def main():
    total_lines = 0
    total_files = 0

    root_dir = Path(sys.argv[1])  # or specify another directory

    for file in root_dir.rglob("*"):
        if file.suffix in {".c", ".h"} and file.is_file():
            total_lines += process_file(file)
            total_files += 1

    print("\n✅ Total files processed:", total_files)
    print("📊 Total lines in all .c and .h files:", total_lines)
    print()

if __name__ == "__main__":
    main()
