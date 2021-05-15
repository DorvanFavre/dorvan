import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/domain/entity/language.dart';
import 'package:enum_to_string/enum_to_string.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'json_text_provider.dart';

final translatedTextProvider = Provider<dynamic>((ref) {
  final language = ref.watch(languageStateProvider).state;
  final texte = ref.watch(jsonTextProvider);
  if (texte.data == null) {
    return {};
  } else {
    return texte.data!.value[EnumToString.convertToString(language)] ??
        texte.data!.value['FR'];
  }
});
