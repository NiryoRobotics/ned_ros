import os
from googletrans import Translator

translator = Translator()


# translation = translator.translate("Hello", src="en", dest="fr")
# print(f"{translation.origin} ({translation.src}) --> {translation.text} ({translation.dest})")

def find_file(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name)


folder_path = "/home/niryo/catkin_ws/src/sphinx_doc/locale/fr/"
file = 'quick_start.po'
file_path = find_file(file, folder_path)


def split_str(sentance, char_nb_limit=60):
    splitted_sentence = []
    piece_of_sentence = ""
    for word in sentance.split():
        if len(piece_of_sentence) >= char_nb_limit:
            splitted_sentence.append('"{}"'.format(piece_of_sentence))
            piece_of_sentence = word
        elif len(piece_of_sentence) == 0:
            piece_of_sentence = word
        else:
            piece_of_sentence += ' ' + word

    splitted_sentence.append('"{}"'.format(piece_of_sentence))

    return '\n'.join(splitted_sentence)


trad_sentence = ""
read_trad = False
overall_translation = ''
num_lines = sum(1 for line in open(file_path))

with open(file_path, 'r', encoding="utf-8") as source_file:
    cpt = 0
    while True:
        line = source_file.readline()
        cpt += 1
        if cpt % 50 == 0:
            print("{}/{}".format(cpt, num_lines))

        if not line:
            break

        if line.startswith('msgid'):
            trad_sentence = line.replace('msgid "', '').rstrip("\n").rstrip('"')
            read_trad = True
        elif line.startswith('msgstr'):
            read_trad = False
            if line.startswith('msgstr ""'):
                next_line = source_file.readline()
                if next_line == "\n":
                    print(trad_sentence)
                    translation = translator.translate(trad_sentence, dest="fr").text
                    print(translation)
                    overall_translation += 'msgstr {}\n\n'.format(split_str(translation))
                else:
                    overall_translation += line + next_line
                continue

        elif read_trad:
            trad_sentence += line.replace('"', '').rstrip("\n")

        overall_translation += line

with open(file_path, 'w', encoding="utf-8") as output_file:
    output_file.write(overall_translation)
