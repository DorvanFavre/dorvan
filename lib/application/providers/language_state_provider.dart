import 'package:dorvan/domain/entity/language.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

final languageStateProvider = StateProvider<Language>((ref) {
  return Language.FR;
});
